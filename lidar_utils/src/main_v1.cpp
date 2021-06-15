/*
This funtion use ros waitForMessage to receive lidar data
*/

#include "ros/ros.h"
#include "lidar_utils/obstacle_detector.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_detctor");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<lidar_utils::Obstacle>("obstacle", 2);
    
    ObstacleDetector mylidar {};
    mylidar.read_parameter();

    ros::Rate loop_rate(mylidar.m_rate);
    int counter {0};
    sensor_msgs::LaserScan::ConstPtr laserPtr;
    lidar_utils::Obstacle::Ptr obsPtr(new lidar_utils::Obstacle);
    while ( ros::ok() ){
        // get the lasted Laser data
        //sensor_msgs::LaserScan::ConstPtr laserPtr; 
        laserPtr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n);
        
        // generate obstacle message and publish
        //lidar_utils::Obstacle::Ptr obsPtr(new lidar_utils::Obstacle);
        mylidar.generate_obstacle_msg(laserPtr, obsPtr);
        pub.publish(*obsPtr);

        ROS_INFO("obstacle broadcast: [%d], [%.3f, %.3f]", counter, obsPtr->dist_ave[0], obsPtr->dist_ave[1]);
        ++counter;
        loop_rate.sleep();
    }    
    
    return 0;
}