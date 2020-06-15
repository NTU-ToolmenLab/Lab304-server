#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <ros/types.h>
#include <math.h>
#include <boost/thread/mutex.hpp>
#include <fstream>
#include <signal.h>
//#include <compal_agv/version.h>
#include <string>

#define ROBOT_POSE_FILE                "/tmp/amcl_pose.yaml"

#define PUB_HZ                        40.0

#define MAX_VX                        1.0
#define MAX_VY                        1.0
#define MAX_VTH                       M_PI

#define LEFT_ENCODE_NAME              "motor/odom_L"
#define RIGHT_ENCODE_NAME             "motor/odom_R"
#define MILEAGE                       "compal_debug/mileage"
#define RECORD_MAP_FLOOR              "map_floor"
#define VERSION_PACKAGE_NAME          "MY_ODOM"
#define VERSIOM_NUMBER                "1.0.2"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define ENCODE_IS_INIT() (g_left_encode_is_init && g_right_encode_is_init)

#define INVALID_VELOCITY(vx, vy, vth) (abs(vx) > MAX_VX || abs(vy) > MAX_VY || abs(vth) > MAX_VTH)

boost::mutex mtx_left;
boost::mutex mtx_right;

/* PARAMETERS */
double g_cnt_per_cycle  = 0;
double g_wheel_radius   = 0;
double g_wheel_distance = 0;

double g_distance_per_cnt = 0;
double g_angle_per_cnt_r  = 0;
double g_angle_per_cnt_R  = 0;

std_msgs::Float64 g_pub_mileage;
std::string g_map_floor = "1";

std::string g_frame_id_odom = "odom";
std::string g_frame_id_base_link_tf = "base_link";
std::string g_frame_id_base_link_pub = "base_link";

int32_t g_left_encode = 0;
int32_t g_right_encode = 0;

int g_left_encode_recv_count = 0;
int g_right_encode_recv_count = 0;

bool g_left_encode_is_init = false;
bool g_right_encode_is_init = false;
bool g_enable_write_pose = true;
bool g_enable_static_period = true;
int g_encode_recv_count = 0;
double g_static_period_num = 0.1;

tf::StampedTransform g_transform;
std::string g_pose_file;

ros::Time current_time, last_time;

void write_position(void) {

  std::ofstream pos_file;

  std::string temp_pose_file = g_pose_file + "~";

  pos_file.open(temp_pose_file.c_str());
  pos_file << "initial_pose_x: " << g_transform.getOrigin().x() << "\n";
  pos_file << "initial_pose_y: " << g_transform.getOrigin().y() << "\n";
  pos_file << "initial_pose_a: " << tf::getYaw(g_transform.getRotation()) << "\n";
  pos_file << "total_milege_distance: " << g_pub_mileage.data << "\n";
  pos_file << "floor: " << g_map_floor << "\n";
  pos_file.close();

  if (std::rename(temp_pose_file.c_str(),g_pose_file.c_str()) != 0)
    ROS_ERROR( "Error renaming file %s -> %s",temp_pose_file.c_str(),g_pose_file.c_str() );
}

void sigint_handler(int sig)
{
  write_position();

  ROS_INFO_STREAM("[MY_ODOM] Save AGV pose("
    << g_transform.getOrigin().x() << ", "
    << g_transform.getOrigin().y() << ", "
    << tf::getYaw(g_transform.getRotation()) << ") , map_floor: "
    << g_map_floor << " and total_mileage: "
    << g_pub_mileage.data << " to file: " << g_pose_file);

  ros::shutdown();
}

void get_map_floor(const std_msgs::String::ConstPtr& sMapfloor)
{
   g_map_floor = sMapfloor->data;
}

void encode_left_recv(const std_msgs::Int32& value)
{
  //mtx_left.lock();
  g_left_encode = value.data;
  //mtx_left.unlock();

  g_left_encode_is_init = true;
  g_left_encode_recv_count ++;
}


void encode_right_recv(const std_msgs::Int32& value)
{
  //mtx_right.lock();
  g_right_encode = value.data;
  //mtx_right.unlock();

  g_right_encode_is_init = true;
  g_right_encode_recv_count++;
}

void calculate_robot_move(int left_encode_diff, int right_encode_diff, double &dx, double &dy, double &dth) {

    int32_t count = 0;
    int      diff_count = 0;
    double concentric_radius=0.0;
    double ratio = 0.0;

    double left_move_distance = 0.0;
    double right_move_distance = 0.0;

    left_move_distance = left_encode_diff * g_distance_per_cnt;
    right_move_distance = right_encode_diff * g_distance_per_cnt;

    if (left_move_distance == right_move_distance)
    {
        dx = left_move_distance;
        dy = 0.0;
        dth = 0.0;
    }
    else
    {
        dth = (right_move_distance - left_move_distance)/g_wheel_distance;
        concentric_radius = (right_move_distance + left_move_distance)/(2*dth);
        dx = concentric_radius * sin(dth);
        dy = concentric_radius * (cos(dth)-1);
    }
   
}


int main(int argc, char** argv){

    ros::init(argc, argv, "odometry_publisher", ros::init_options::NoSigintHandler);
    signal(SIGINT, sigint_handler);

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener listener;

    ros::Subscriber encoder_left_sub = n.subscribe(LEFT_ENCODE_NAME, 1000, &encode_left_recv);
    ros::Subscriber encoder_right_sub = n.subscribe(RIGHT_ENCODE_NAME, 1000, &encode_right_recv);
    ros::Subscriber record_map_floor_sub = n.subscribe<std_msgs::String>(RECORD_MAP_FLOOR, 1 , get_map_floor);
    ros::Publisher  milege_pub = n.advertise<std_msgs::Float64>(MILEAGE , 50);

    double total_x = 0.0;
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double dx = 0.0;
    double dy = 0.0;
    double dth = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;

    int32_t last_left_encode;
    int32_t last_right_encode;

    int left_encode_diff = 0;
    int right_encode_diff = 0;

    uint32_t tick = 0;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(PUB_HZ);
    ros::Rate r_wait(200);

    ros::NodeHandle nh_private("~");

    /* Get parameters */
    nh_private.param("cnt_per_cycle",   g_cnt_per_cycle,  714.0);
    nh_private.param("wheel_radius",    g_wheel_radius,   0.04);
    nh_private.param("wheel_distance",  g_wheel_distance, 0.29);

    nh_private.param("frame_id_odom",  g_frame_id_odom, std::string("odom"));
    nh_private.param("frame_id_base_link_tf",  g_frame_id_base_link_tf, std::string("base_link"));
    nh_private.param("frame_id_base_link_pub",  g_frame_id_base_link_pub, std::string("base_link"));
    nh_private.param("pose_file_param", g_pose_file,  std::string(ROBOT_POSE_FILE));
    nh_private.param("total_milege_distance" , g_pub_mileage.data , 0.0 );
    nh_private.param("enable_write_pose", g_enable_write_pose, true);
    nh_private.param("enable_static_period", g_enable_static_period, false);
    nh_private.param("static_period_num", g_static_period_num, 0.1);
    nh_private.param("encoder_recv_num", g_encode_recv_count, 2);
    //compal_common::VersionHelper version(VERSION_PACKAGE_NAME, VERSIOM_NUMBER);

    g_distance_per_cnt = ( ( 2 * g_wheel_radius * M_PI ) / g_cnt_per_cycle );
    g_angle_per_cnt_R  = ( g_distance_per_cnt / g_wheel_distance );
    g_angle_per_cnt_r  = ( g_distance_per_cnt / (g_wheel_distance/2) );

    ROS_INFO_STREAM( "MY_ODOM parameter list:");
    ROS_INFO_STREAM( "  - cnt_per_cycle  = " << g_cnt_per_cycle);
    ROS_INFO_STREAM( "  - wheel_radius   = " << g_wheel_radius);
    ROS_INFO_STREAM( "  - wheel_distance = " << g_wheel_distance);

    ROS_INFO_STREAM( "  - frame_id_odom = " << g_frame_id_odom);
    ROS_INFO_STREAM( "  - frame_id_base_link_tf = " << g_frame_id_base_link_tf);
    ROS_INFO_STREAM( "  - frame_id_base_link_pub = " << g_frame_id_base_link_pub);
    ROS_INFO_STREAM( "  - g_enable_write_pose = " << g_enable_write_pose);
    ROS_INFO_STREAM("  - enable_static_period = " << g_enable_static_period);
    ROS_INFO_STREAM("  - static_period_num = " << g_enable_static_period);
    ROS_INFO_STREAM( "  - encoder_recv_num = " << g_encode_recv_count);

    ros::spinOnce();

    while (!ENCODE_IS_INIT() && n.ok()) {
        ros::Duration(2.0).sleep();
        ROS_WARN_STREAM( "[MY_ODOM] Encode hasn't been updated yet...plz check topic "
                         << RIGHT_ENCODE_NAME << " and "
                         << LEFT_ENCODE_NAME);
        ros::spinOnce();
    }

    last_left_encode  = g_left_encode;
    last_right_encode = g_right_encode;

    ROS_INFO_STREAM( "[MY_ODOM] Encode is recv!! MY_ODOM is ready to go.");

    while (n.ok()) 
    {
        dx = 0.0;
        dy = 0.0;
        dth = 0.0;

        vx = 0.0;
        vy = 0.0;
        vth = 0.0;

        while (!(g_left_encode_recv_count >=g_encode_recv_count 
            && g_right_encode_recv_count >=g_encode_recv_count))
        {
            ros::spinOnce();               // check for incoming messages
            r_wait.sleep();
        }
        current_time = ros::Time::now();
        g_left_encode_recv_count = 0;
        g_right_encode_recv_count = 0;

        right_encode_diff = g_right_encode - last_right_encode;
        left_encode_diff  = g_left_encode  - last_left_encode;

        calculate_robot_move(left_encode_diff, right_encode_diff, dx, dy, dth);

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        if (g_enable_static_period)
            dt = g_static_period_num;
        //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        //double delta_th = vth * dt;

        vx  = dx / dt;
        vy  = dy / dt;
        vth = dth / dt;

	    //ROS_INFO( "[MY_ODOM] VEL = (%0.3f, %0.3f, %0.3f) ENC = (%d, %d) time = %f", vx, vy, vth, left_encode_diff, right_encode_diff, dt);
        //ROS_INFO( "[MY_ODOM] VELOCITY INTERVAL = %f", dt);

        if (INVALID_VELOCITY(vx, vy, vth)) {
            ROS_ERROR( "[MY_ODOM] INVALID VELOCITY = (%f, %f, %f) ENCODE = (%d, %d) INTERVAL = %f",
			           vx, vy, vth, left_encode_diff, right_encode_diff, dt);
            dx = dy = dth = vx = vy = vth = 0.0;
		}

        total_x += dx;

        double delta_x = (dx * cos(th) - dy * sin(th));
        double delta_y = (dx * sin(th) + dy * cos(th));
        double delta_th = dth;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        g_pub_mileage.data += sqrt(pow(delta_x,2) + pow(delta_y,2));
        milege_pub.publish(g_pub_mileage);

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = g_frame_id_odom;
        odom_trans.child_frame_id = g_frame_id_base_link_tf;

        if (vx != 0 || vth != 0) {
            ROS_INFO_STREAM("[MY_ODOM] vx = "<< vx << ", vth =" << vth << ", total_x = " << total_x);
        }

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = g_frame_id_odom;

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = g_frame_id_base_link_pub;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;

        last_left_encode = g_left_encode;
        last_right_encode = g_right_encode;

	if(g_enable_write_pose){
            /* Record robot pose in g_transform, which will be written to file before my_odom terminate */
            if ( listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.01)) ) {
                listener.lookupTransform("/map", "/base_link", ros::Time(0), g_transform);
            }
	}

        r.sleep();
	
	if(g_enable_write_pose){
            /* Write position to file every 0.5 second */
            tick = (++tick)%10;
            if ( (tick % 5) == 0) write_position();
	}
    }
}
