/*
This files implements lidar obstacle detection algorithm. 
After receiving lidar messages, it publishes the self-defined message.
*/

#include "lidar_detector/lidar_detector.h"


void ObstacleDetector::read_parameter(){
    // This functions reads the detection parameter from the param yaml file
    // for future interface...
}


inline bool ObstacleDetector::is_in_filter(const float d){
    if ( d <= m_dmin || d >= m_dmax || std::isnan(d) ){
        return false;
    }
    else{
        return true;
    }
}


bool ObstacleDetector::check_pre(const std::vector< float > &d, int idx){
    // This function
    int count {0};
    for (int i = 0; i < m_split_width; ++i){
        if ( !is_in_filter(d[std::abs(idx-i) % m_length]) ){
            count++;
        }
    }

    if (count == m_split_width){
        return true;
    }
    else{
        return false;
    }
}


bool ObstacleDetector::check_post(const float *d, int idx){
    // This function 
    int count {0};
    for (int i = 0; i < m_split_width; ++i){
        if ( !is_in_filter(d[(idx+i) % m_length]) ){
            count++;
        }
    }

    if (count == m_split_width){
        return true;
    }
    else{
        return false;
    }

}


int ObstacleDetector::find_starting_point(const std::vector< float > &d){
    // This function finds the starting point of distance array
    int start = 0;
    for (int i = 0; i < m_length; ++i){
        // only check non-nan element 
        if ( is_in_filter(d[i]) ){
            if ( check_pre(d, i-1) ){
                start = i;
                break;
            }
        }
    }

    return start;
}


void ObstacleDetector::rearrange_array(const std::vector< float > &d, const std::vector< int > &a, float *dmod, int *amod){
    // This function rearrange the distance and angle arrays so that the first point 
    // is nonzer starting point
    
    // find starting point
    int start {0};
    for (int i = 0; i < m_length; ++i){
        // only check non-nan element 
        if ( is_in_filter(d[i]) ){
            if ( check_pre(d, i-1) ){
                start = i;
                break;
            }
        }
    }


    if (start != 0){
        // define the former half of dmod and amod
        for (int i = 0; i < m_length-start; ++i){
            dmod[i] = d[i+start];
            amod[i] = a[i+start];
        }

        // define the latter half of dmod and amod
       for (int i = m_length-start; i < m_length; ++i){
           dmod[i] = d[i-(m_length-start)];
           amod[i] = a[i-(m_length-start)] + 360;
        }
    }
    else{
        // copy d and a to dmod and amod
        for (int i = 0; i < m_length; ++i){
            dmod[i] = d[i];
            amod[i] = a[i];
        }   
    }  
} 


void ObstacleDetector::split_obstacle(const std::vector< float > &d, const std::vector< int > &a, 
                std::vector< std::vector<float> > &dsub, std::vector< std::vector<int> > &asub){
    // This function splits the distance and angle array into subarrays
    float *dmod = new float[m_length];
    int *amod = new int[m_length];
    rearrange_array(d, a, dmod, amod);

    std::vector<float> dtmp;
    std::vector<int> atmp;
    int i {0};
    while (1){
        if ( is_in_filter(dmod[i]) ){
            dtmp.push_back(dmod[i]);
            atmp.push_back(amod[i]);
            i++;
        }
        else{
            if ( check_post(dmod, i) ){
                dsub.push_back(dtmp);
                asub.push_back(atmp);

                dtmp.clear();
                atmp.clear();

                while ( i < m_length && !is_in_filter(dmod[i]) )
                    i++;
            }
            else{
                i++;
            }
        }

        // stopping criterion
        if ( i >= m_length ){
            if ( !dtmp.empty() || !atmp.empty() ){
                dsub.push_back(dtmp);
                asub.push_back(atmp);
            }
            break;
        } 
    }

    delete[] dmod;
    delete[] amod;
}


void ObstacleDetector::generate_obstacle_msg(const sensor_msgs::LaserScan::ConstPtr &laser, 
        ObstacleDetector::Obstacle::Ptr &obs){
    // This function refines the distance information for each obstacle 
    // by deleting small distance subarray (induced by noise)

    // read distance range
    std::vector< float > d = laser->ranges;
    // create a
    std::vector< int > a;
    for (int i = 0; i < m_length; ++i){
        a.push_back(i);
    }
    std::vector< std::vector<float> > dsub;
    std::vector< std::vector<int> > asub;

    split_obstacle(d, a, dsub, asub);
    
    for (int i = 0; i<dsub.size(); ++i){
        if ( dsub[i].size() >= m_valid_width ){
            float dsum {0};
            int asum {0};
            for (int j = dsub[i].size()-1; j>=0; --j){
                dsum += dsub[i].at(j);
                asum += asub[i].at(j);
            }
            float ave {float(asum) / asub[i].size()};
            
            obs->dist_ave.push_back( dsum / dsub[i].size() );
            obs->angle_ave.push_back( ave - int(ave/360)*360 ) ; 
            obs->angle_span.push_back( int(asub[i].size()) );
            obs->header = laser->header;
        }
    }
}

