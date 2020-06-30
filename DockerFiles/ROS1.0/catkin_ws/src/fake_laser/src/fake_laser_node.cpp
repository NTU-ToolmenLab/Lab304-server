
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define DEG2RAD(x) ((x)*M_PI/180.)
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

void publish_scan(ros::Publisher& pub, 
                  size_t node_count, ros::Time start,
                  double scan_time, bool inverted, 
                  float angle_min, float angle_max, 
                  std::string frame_id)
{
    static int scan_count = 0;
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_count++;

    scan_msg.angle_min =  M_PI - angle_min;
    scan_msg.angle_max =  M_PI - angle_max;
    scan_msg.angle_increment = 
        (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);

    scan_msg.range_min = 0.15;
    scan_msg.range_max = 6.;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);
    if (!inverted) { // assumes scan window at the top
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) 0.0;
            if (read_value == 0.0)
                scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[i] = read_value;
            scan_msg.intensities[i] = (float) 0.0;
        }
    } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)0.0;
            if (read_value == 0.0)
                scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else
                scan_msg.ranges[node_count-1-i] = read_value;
            scan_msg.intensities[node_count-1-i] = (float) 0.0;
        }
    }

    pub.publish(scan_msg);
}

static const std::string kFrameIdKey_ = "frame_id";
static const std::string kMaxAngleKey_ = "angle_max";
static const std::string kMinAngleKey_ = "angle_min";
static const std::string kIsRadKey_ = "is_rad";

int main(int argc, char * argv[]) {

    ros::init(argc, argv, "rplidar_node");

    ros::NodeHandle nh;
    ros::NodeHandle prv_nh("~");
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    std::string _frame_id;
    bool _is_rad;
    float angle_min;
    float angle_max;
    prv_nh.param<std::string>(kFrameIdKey_, _frame_id, "laser");
    prv_nh.param<bool>(kIsRadKey_, _is_rad, false);
    if (_is_rad) {
        prv_nh.param<float>(kFrameIdKey_, angle_max, 0.0);
        prv_nh.param<float>(kFrameIdKey_, angle_min, 3.14);
    } else {
        prv_nh.param<float>(kFrameIdKey_, angle_max, 0.0);
        prv_nh.param<float>(kFrameIdKey_, angle_min, 359.0);
        angle_min = DEG2RAD(angle_min);
        angle_max = DEG2RAD(angle_max);
    }

    size_t count = 360;

    bool inverted = false;

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;

    ros::Rate rate(15);

    while (ros::ok()) {

        start_scan_time = ros::Time::now();
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

        publish_scan(scan_pub, count, 
                      start_scan_time, scan_duration, inverted,  
                      angle_min, angle_max, 
                      _frame_id);
        rate.sleep();
    }

    return 0;
}
