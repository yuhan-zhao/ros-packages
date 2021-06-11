#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "obstacle_detector/Obstacle.h"

#include <cmath>
#include <vector>
#include <numeric>


class ObstacleDetector{
public: 
    int m_valid_width;
    int m_split_width;
    int m_length;
    double m_dmin;
    double m_dmax;

public:
    ObstacleDetector(double dmin=0.0, double dmax=3.5, int valid_wid=3, int split_wid=5, int len=360) 
        : m_dmin{dmin}, m_dmax{dmax}, m_valid_width{valid_wid}, m_split_width{split_wid}, m_length{len}
    {
    }
    void read_parameter();
    bool is_in_filter(const float);
    bool check_pre(const std::vector< float > &, int);
    bool check_post(const float *, int);
    int find_starting_point(const std::vector< float > &);
    void rearrange_array(const std::vector< float > &, const std::vector< int > &, float *, int *);
    void split_obstacle(const std::vector< float > &, const std::vector< int > &, 
                std::vector< std::vector<float> > &, std::vector< std::vector<int> > &);
    void generate_obstacle_msg(const sensor_msgs::LaserScan::ConstPtr &, 
            lidar_detector::Obstacle::Ptr &);
};

#endif