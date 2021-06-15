/*
This funtion use traditional callback function to receive lidar data
*/
#include "ros/ros.h"
#include "lidar_utils/obstacle_detector.h"


void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &laserPtr){
    ros::Rate loop_rate(1);

    double dmin {0.3};
    double dmax {2};
    int valid_wid {3};
    int split_wid {5};
    int len {360};
    int counter {0};
    ObstacleDetector mylidar {dmin, dmax, valid_wid, split_wid, len};
    //mylidar.read_parameter();

    lidar_utils::Obstacle::Ptr obsPtr(new lidar_utils::Obstacle);
    mylidar.generate_obstacle_msg(laserPtr, obsPtr);

    ROS_INFO("obstacle info broadcasted.", counter);
    loop_rate.sleep();
}


int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_detctor_v2");
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

    /* use 1 as the queue length. If the recieved message is greater than queue length,
       then we throw the old message when new one arrives.
       This happens when processing callback is slow. If call back is fast, 
       no new message is needed to store in the queue.
    */
    sub = n.subscribe("/scan", 1, lidarCallback);
    ros::spin();
  
    return 0;
}
