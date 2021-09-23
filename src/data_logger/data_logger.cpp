#include "data_logger.hpp" // header in local directory
#include <iostream> // header in standard library
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <fstream>

using namespace DataLogger;

SubscriptionLogger::SubscriptionLogger(std::string topic_name)
{
    topic = topic_name;
    recv_counter = 0;
}

// according to this https://github.com/irobot-ros/ros2-performance/tree/master/performances/irobot-benchmark
/*
ubscriptionLogger::get_late(float period)
{
    
        min(0.2*period, 5ms)
    
}

SubscriptionLogger::get_too_late(float period)
{
    
        max(period, 50ms)
    
}
*/
SubscriptionLogger::get_mean()
{
    double average = std::accumulate(time_diffs.begin(), time_diffs.end(), 0.0) / time_diffs.size();
    return average;
}

// https://stackoverflow.com/questions/9874802/how-can-i-get-the-maximum-or-minimum-value-in-a-vector
SubscriptionLogger::get_max_latency()
{
    double max_latency = *std::max_element(time_diffs.begin(), time_diffs.end());
    return max_latency;
}

SubscriptionLogger::get_min_latency()
{
    double min_latency = *std::min_element(time_diffs.begin(), time_diffs.end());
    return min_latency;
}

int save_logged_data(int mode, std::string filename)
{
    
    switch(mode)
    {
        case 1:

    }
}