#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <ros/types.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>
#include <fstream>
#include <signal.h>

#define ROBOT_POSE_FILE                "/tmp/amcl_pose.yaml"

#define PUB_HZ                        10.0

#define MAX_VX                        1.0
#define MAX_VY                        1.0
#define MAX_VTH                       M_PI

#define LEFT_ENCODE_NAME              "motor/odom_L"
#define RIGHT_ENCODE_NAME             "motor/odom_R"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define INVALID_VELOCITY(vx, vy, vth) (abs(vx) > MAX_VX || abs(vy) > MAX_VY || abs(vth) > MAX_VTH)

boost::mutex mtx_left;
boost::mutex mtx_right;

/* PARAMETERS */
double g_cnt_per_cycle  = 0;
double g_wheel_radius   = 0;
double g_wheel_distance = 0;
bool   g_enable_amcl = 0;
bool   g_enable_encode = 0;

double g_distance_per_cnt = 0;
double g_angle_per_cnt_r  = 0;
double g_angle_per_cnt_R  = 0;

int32_t g_left_encode = 0;
int32_t g_right_encode = 0;

std::string g_frame_id_odom = "odom";
std::string g_frame_id_base_link_tf = "base_link";
std::string g_frame_id_base_link_pub = "base_link";

bool g_left_encode_is_init = false;
bool g_right_encode_is_init = false;

#define ENCODE_IS_INIT() (g_left_encode_is_init && g_right_encode_is_init)

double g_cmd_vel_x;
double g_cmd_vel_z;

tf::StampedTransform g_transform;
std::string g_pose_file;

void write_position(void) {

  std::ofstream pos_file;

  std::string temp_pose_file = g_pose_file + "~";

  pos_file.open(temp_pose_file.c_str());
  pos_file << "initial_pose_x: " << g_transform.getOrigin().x() << "\n";
  pos_file << "initial_pose_y: " << g_transform.getOrigin().y() << "\n";
  pos_file << "initial_pose_a: " << tf::getYaw(g_transform.getRotation()) << "\n";
  pos_file.close();

  if (std::rename(temp_pose_file.c_str(),g_pose_file.c_str()) != 0)
    ROS_ERROR( "Error renaming file%s -> %s",temp_pose_file.c_str(),g_pose_file.c_str() );
}

void sigint_handler(int sig)
{
  std::ofstream pos_file;

  write_position();

  ROS_INFO_STREAM("[MY_ODOM] Save pos("
    << g_transform.getOrigin().x() << ", "
    << g_transform.getOrigin().y() << ", "
    << tf::getYaw(g_transform.getRotation()) << ") to file: " << g_pose_file);

  ros::shutdown();
}

void cmd_vel_recv(const geometry_msgs::Twist& msg)
{
  g_cmd_vel_x = msg.linear.x;
  g_cmd_vel_z = msg.angular.z;
}

void encode_left_recv(const std_msgs::Int32& value)
{
  mtx_left.lock();
  g_left_encode = value.data;
  mtx_left.unlock();

  g_left_encode_is_init = true;
}


void encode_right_recv(const std_msgs::Int32& value)
{
  mtx_right.lock();
  g_right_encode = value.data;
  mtx_right.unlock();

  g_right_encode_is_init = true;
}

void mimic_robot_encode(double cmd_vel_x, double cmd_vel_z, double wheel_dist, double dist_per_cnt, int32_t left_encode, int32_t right_encode, ros::Publisher &encode_left_pub, ros::Publisher &encode_right_pub)
{
  double left_move_dist, right_move_dist;
  double move_dist = cmd_vel_x / PUB_HZ;
  double move_angle = cmd_vel_z / PUB_HZ;
  std_msgs::Int32 encode;

  if (cmd_vel_x == 0.0 && cmd_vel_z != 0) {

    /* In-place Rotation */
    left_move_dist  = -(wheel_dist * move_angle / 2);
    right_move_dist =  (wheel_dist * move_angle / 2);

  } else if (cmd_vel_x != 0.0 && cmd_vel_z == 0.0) {

    /* Move forward */
    right_move_dist = left_move_dist = move_dist;

  } else if (cmd_vel_x != 0.0 && cmd_vel_z != 0.0) {

    /* Move in a curve */
    left_move_dist  = move_dist - (wheel_dist * move_angle / 2);
    right_move_dist = move_dist + (wheel_dist * move_angle / 2);

  }

  //publish the encoder
  encode.data = left_encode + left_move_dist / dist_per_cnt;
  encode_left_pub.publish(encode);
  encode.data = right_encode + right_move_dist / dist_per_cnt;
  encode_right_pub.publish(encode);
}

void mimic_robot_move(double cmd_vel_x, double cmd_vel_z, double &dx, double &dth) {
    dx = cmd_vel_x / PUB_HZ;
    dth = cmd_vel_z / PUB_HZ;
}

void calculate_robot_move(int left_encode_diff, int right_encode_diff, double &dx, double &dth) {

    int32_t count = 0;
    int      diff_count = 0;
    double concentric_radius=0.0;
    double ratio = 0.0;


   //ROS_WARN_STREAM("left_encode_diff:"<<left_encode_diff<<"  right_encode_diff:"<<right_encode_diff);
    if ( left_encode_diff > 0 && right_encode_diff < 0 )
    {
        count = (abs(left_encode_diff)+abs(right_encode_diff))/2;
        dth = -count * g_angle_per_cnt_r;
        dx = 0;
        //ROS_INFO_STREAM("Right Rotation!! >0 <0");
    }
    else if ( left_encode_diff < 0 && right_encode_diff >0)
    {
        count = (abs(left_encode_diff)+abs(right_encode_diff))/2;
        dth = count * g_angle_per_cnt_r;
        dx = 0;
        //ROS_INFO_STREAM("Left Rotation!! <0 >0");
    }
    else if (right_encode_diff==0 || left_encode_diff==0)
    {
        //ROS_WARN_STREAM("Left or Right ==0");
        if (right_encode_diff>0)
        {
            count = right_encode_diff;
            //ROS_INFO_STREAM("Right>0 Left==0");
        }
        else if (left_encode_diff>0)
        {
            count = left_encode_diff;
            //ROS_WARN_STREAM("Left>0  Right==0");
        }
        else if (right_encode_diff<0)
        {
            count = right_encode_diff;
            //ROS_INFO_STREAM("Right<0 Left==0");
        }
        else if (left_encode_diff<0)
        {
            count = left_encode_diff;
            //ROS_INFO_STREAM("Left<0  Right==0");
        }
        else
            count =0;
            //ROS_INFO_STREAM("count:"<<count);
            dth = count* g_angle_per_cnt_R;
            dx = (g_wheel_distance/2)*sin(dth);
    }
    else if ( left_encode_diff < 0 && right_encode_diff < 0)
    {

        if (abs(left_encode_diff) > abs(right_encode_diff))
        {
            //ROS_INFO_STREAM("Turn Right!!");
            ratio = double(left_encode_diff)/double(right_encode_diff);
            concentric_radius = (g_wheel_distance) / (ratio-1);
            dth = (abs(right_encode_diff) * g_distance_per_cnt)/concentric_radius;
            dx = (concentric_radius+(g_wheel_distance/2))*sin(-dth);
        }

        else if (abs(right_encode_diff) > abs(left_encode_diff))
        {
            //ROS_INFO_STREAM("Turn Left!!");
            ratio = double(right_encode_diff)/double(left_encode_diff);
            concentric_radius = (g_wheel_distance) / (ratio-1);
            dth = -(abs(left_encode_diff) * g_distance_per_cnt)/concentric_radius;
            dx = (concentric_radius+(g_wheel_distance/2))*sin(dth);
        }
        else if (left_encode_diff == right_encode_diff)
        {
            //ROS_INFO_STREAM("Backward!!");
            count = (abs(left_encode_diff) + abs(right_encode_diff))/2;
            dth = 0;
            dx = -count * g_distance_per_cnt;
        }
    }

    else if (right_encode_diff >0 && left_encode_diff >0)
    {
        if (left_encode_diff > right_encode_diff)
        {
            //ROS_INFO_STREAM("Turn Right!!");
            ratio = double(left_encode_diff)/double(right_encode_diff);
            concentric_radius = (g_wheel_distance) / (ratio-1);
            dth = -(right_encode_diff * g_distance_per_cnt)/concentric_radius;
            dx = (concentric_radius+(g_wheel_distance/2))*sin(-dth);
        }
        else if (right_encode_diff > left_encode_diff)
        {
            //ROS_INFO_STREAM("Turn Left!!");
            ratio = double(right_encode_diff)/double(left_encode_diff);
            concentric_radius = (g_wheel_distance) / (ratio-1);
            dth = (left_encode_diff * g_distance_per_cnt)/concentric_radius;
            dx = (concentric_radius+(g_wheel_distance/2))*sin(dth);
        }
        else if (left_encode_diff == right_encode_diff)
        {
            //ROS_INFO_STREAM("Foward!!");
            count = (left_encode_diff + right_encode_diff)/2;
            dth = 0;
            dx = count * g_distance_per_cnt;
        }
    }

    if (left_encode_diff != 0 || right_encode_diff != 0)
    {
        /*ROS_INFO_STREAM( "ROBOT MOVE (" << "left_diff = "  << left_encode_diff  << ", "
                                        << "right_diff = " << right_encode_diff << ", "
                                        << "dx = "         << dx << ", "
                                        << "dth = "        << dth << ")");*/
    }
}


int main(int argc, char** argv){

    ros::init(argc, argv, "odometry_publisher", ros::init_options::NoSigintHandler);
    signal(SIGINT, sigint_handler);

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener listener;

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 100, &cmd_vel_recv);
    ros::Publisher encode_left_pub = n.advertise<std_msgs::Int32>(LEFT_ENCODE_NAME, 1000);
    ros::Publisher encode_right_pub = n.advertise<std_msgs::Int32>(RIGHT_ENCODE_NAME, 1000);

    ros::Subscriber encode_left_sub = n.subscribe(LEFT_ENCODE_NAME, 1000, &encode_left_recv);
    ros::Subscriber encode_right_sub = n.subscribe(RIGHT_ENCODE_NAME, 1000, &encode_right_recv);

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

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(PUB_HZ);

    ros::NodeHandle nh_private("~");

    /* Get parameters */
    nh_private.param("cnt_per_cycle",   g_cnt_per_cycle,  714.0);
    nh_private.param("wheel_radius",    g_wheel_radius,   0.04);
    nh_private.param("wheel_distance",  g_wheel_distance, 0.29);
    nh_private.param("enable_encode",   g_enable_encode,  false);
    nh_private.param("enable_amcl",     g_enable_amcl,    false);
    nh_private.param("pose_file_param", g_pose_file,      std::string(ROBOT_POSE_FILE));

    nh_private.param("frame_id_odom",  g_frame_id_odom, std::string(g_enable_amcl ? "odom" : "map"));
    nh_private.param("frame_id_base_link_tf",  g_frame_id_base_link_tf, std::string("base_link"));
    nh_private.param("frame_id_base_link_pub",  g_frame_id_base_link_pub, std::string("base_link"));

    g_distance_per_cnt = ( ( 2 * g_wheel_radius * M_PI ) / g_cnt_per_cycle );
    g_angle_per_cnt_R  = ( g_distance_per_cnt / g_wheel_distance );
    g_angle_per_cnt_r  = ( g_distance_per_cnt / (g_wheel_distance/2) );

    ROS_INFO_STREAM( "MY_ODOM parameter list:");
    ROS_INFO_STREAM( "  - cnt_per_cycle  = " << g_cnt_per_cycle);
    ROS_INFO_STREAM( "  - wheel_radius   = " << g_wheel_radius);
    ROS_INFO_STREAM( "  - wheel_distance = " << g_wheel_distance);
    ROS_INFO_STREAM( "  - enable_encode = "  << g_enable_encode);
    ROS_INFO_STREAM( "  - enable_amcl = "    << g_enable_amcl);

    ROS_INFO_STREAM( "  - frame_id_odom = " << g_frame_id_odom);
    ROS_INFO_STREAM( "  - frame_id_base_link_tf = " << g_frame_id_base_link_tf);
    ROS_INFO_STREAM( "  - frame_id_base_link_pub = " << g_frame_id_base_link_pub);

    //while (!ENCODE_IS_INIT()) {
    //    ros::Duration(2.0).sleep();
    //    ROS_WARN_STREAM( "[MY_ODOM] Encode hasn't been updated yet...plz check topic "
    //                     << RIGHT_ENCODE_NAME << " and "
    //                     << LEFT_ENCODE_NAME);
    //    ros::spinOnce();
    //}

    ros::spinOnce();

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

        ros::spinOnce();               // check for incoming messages

        if (g_cmd_vel_x != 0 || g_cmd_vel_z != 0) {
          //ROS_INFO_STREAM( "======================================================================");
          //ROS_INFO_STREAM( "[MY_ODOM] cmd_vel_x : " << g_cmd_vel_x   << " cmd_vel_z : " << g_cmd_vel_z);
        }

        if (g_enable_encode) {

          mimic_robot_encode(g_cmd_vel_x, g_cmd_vel_z, g_wheel_distance, g_distance_per_cnt, g_left_encode, g_right_encode, encode_left_pub, encode_right_pub);

          ros::spinOnce();               // check for incoming messages

          mtx_left.lock();
          left_encode_diff  = g_left_encode  - last_left_encode;
          mtx_left.unlock();

          mtx_right.lock();
          right_encode_diff = g_right_encode - last_right_encode;
          mtx_right.unlock();

          if (left_encode_diff != 0 || right_encode_diff != 0) {
            //ROS_INFO_STREAM( "[MY_ODOM] encode_l : " << left_encode_diff << " encode_r : " << right_encode_diff);
          }

          calculate_robot_move(left_encode_diff, right_encode_diff, dx, dth);

          if (dx != 0 || dth != 0) {
            //ROS_INFO_STREAM( "[MY_ODOM] dx : " << dx          << " dth : " << dth);
          }

        } else {

          mimic_robot_move(g_cmd_vel_x, g_cmd_vel_z, dx, dth);

        }

        current_time = ros::Time::now();
        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        //double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        //double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        //double delta_th = vth * dt;

        vx  = dx / dt;
        vy  = dy / dt;
        vth = dth / dt;

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

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = g_frame_id_odom;
        odom_trans.child_frame_id = g_frame_id_base_link_tf;

        //if (x != 0 || y != 0 || th != 0) {
        //    ROS_INFO_STREAM("[MY_ODOM] tf_x = "<< x << ", tf_y = " << y << ", tf_th = " << th);
        //}

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

        if (vx != 0 || vth != 0) {
            //ROS_INFO_STREAM("[MY_ODOM] vx = "<< vx << ", vth = " << vth << ", total_x = " << total_x);
        }

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

        /* Record robot pose in g_transform, which will be written to file before my_odom terminate */
        if ( listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.01)) ) {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), g_transform);
        }

        r.sleep();

        /* Write position to file every 0.5 second */
        tick = (++tick)%10;
        if ( (tick % 5) == 0) write_position();
    }
}
