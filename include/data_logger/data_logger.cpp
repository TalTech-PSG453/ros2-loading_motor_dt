#include "data_logger.hpp" // header in local directory
#include <iostream> // header in standard library
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <fstream>

DataLogger::SubscriptionLogger::SubscriptionLogger(std::string topic_name)
{
    topic = topic_name;
    recv_counter = 0;
}

DataLogger::PublisherLogger::PublisherLogger(std::string topic_name)
{
    topic = topic_name;
    sent_counter = 0;
}

// according to this https://github.com/irobot-ros/ros2-performance/tree/master/performances/irobot-benchmark
/*
SubscriptionLogger::get_late(float period)
{
    
        min(0.2*period, 5ms)
    
}

SubscriptionLogger::get_too_late(float period)
{
    
        max(period, 50ms)
    
}
*/
uint32_t DataLogger::SubscriptionLogger::get_mean()
{
    uint32_t average = std::accumulate(time_diffs.begin(), time_diffs.end(), 0.0) / time_diffs.size();
    return average;
}

// https://stackoverflow.com/questions/9874802/how-can-i-get-the-maximum-or-minimum-value-in-a-vector
uint32_t DataLogger::SubscriptionLogger::get_max_latency()
{
    double max_latency = *std::max_element(time_diffs.begin(), time_diffs.end());
    return max_latency;
}

uint32_t DataLogger::SubscriptionLogger::get_min_latency()
{
    double min_latency = *std::min_element(time_diffs.begin(), time_diffs.end());
    return min_latency;
}

int DataLogger::SubscriptionLogger::save_logged_data(std::string filename)
{
    
    std::ofstream RecordedFile;
    RecordedFile.open("~/dev_ws/src/loading_motor_dt/recorded_data/"+filename);
    RecordedFile << "topic,received(#)\n";
    RecordedFile << topic << "," << recv_counter<<"\n"; 
    RecordedFile.close();

    return 1;

}
int DataLogger::PublisherLogger::save_logged_data(std::string filename)
{
    
    std::ofstream RecordedFile;
    RecordedFile.open("~/dev_ws/src/loading_motor_dt/recorded_data/"+filename);
    RecordedFile << "topic,sent(#)\n";
    RecordedFile << topic << "," << sent_counter<<"\n"; 
    RecordedFile.close();

    return 1;

}