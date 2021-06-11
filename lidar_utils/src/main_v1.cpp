#include "obstacle_detector/obstacle_detector.h"
//#include "ros/ros.h"
//#include "sensor_msgs/LaserScan.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_detctor");
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Rate loop_rate(1);     // control loop frequency, Hz

    double dmin {0.3};
    double dmax {2};
    int valid_wid {3};
    int split_wid {5};
    int len {360};
    int counter {0};
    ObstacleDetector mylidar {dmin, dmax, valid_wid, split_wid, len};
    //mylidar.read_parameter();

    while ( ros::ok() ){
        // get the lasted Laser data
        sensor_msgs::LaserScan::ConstPtr laserPtr; 
        laserPtr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/chatter", n);
        obstacle_detector::Obstacle::Ptr obsPtr(new lidar_detector::Obstacle);
        
        mylidar.generate_obstacle_msg(laserPtr, obsPtr);

        ROS_INFO("obstacle broadcast: [%d], [%.3f, %.3f]", counter, obsPtr->dist_ave[0], obsPtr->dist_ave[1]);
        ++counter;
        loop_rate.sleep();
    }    
    
    return 0;
}