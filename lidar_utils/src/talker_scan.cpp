#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdlib>


typedef std_msgs::String String;
typedef sensor_msgs::LaserScan LaserScan;


std::vector<float> read_lidar_data(char **argv){
    //std::ifstream dataFile("lidar_data.txt");
    std::ifstream dataFile(argv[1]);
	std::vector<float> data;
    if ( dataFile.is_open() ){
        std::string line;
        std::getline(dataFile, line);
        
        std::stringstream ss(line);
        std::string::size_type sz;     // alias of size_t
        while ( ss.good() ){
            std::string substr;
            std::getline(ss, substr, ',');
            data.push_back( std::stof(substr, &sz) );
        }
    }
    dataFile.close();

    // print result
    std::cout << data.size() << '\n';
	for(int i = 0; i<data.size(); ++i) {    //print all splitted strings
        std::cout << data.at(i) << '\t';
    }
    std::cout << '\n';

    return data;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<LaserScan>("scan", 10);
	ros::Rate loop_rate(2);
			    
	std::uint32_t count {0};
	std::vector<float> data = read_lidar_data(argv);
	
	while (ros::ok()){
/*		String msg;
		float ff {1.1111};
		std::stringstream ss;
		ss << "hello" << count << ff;
		msg.data = ss.str();		
*/
		LaserScan msg;
		msg.header.seq = count;
		msg.ranges = data;

		ROS_INFO("Laser data sent %d", count);
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
