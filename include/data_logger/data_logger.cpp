#include "data_logger.hpp" // header in local directory
#include <iostream> // header in standard library
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <fstream>

/* careful with this one*/
using namespace DataLogger;

std::vector<PublisherLogger*> PublisherLogger::loggerList;
std::vector<SubscriptionLogger*> SubscriptionLogger::loggerList;

SubscriptionLogger::SubscriptionLogger(std::string topic_name)
{
    /* fix this to smart pointers */
    topic = topic_name;
    recv_counter = 0;
    loggerList.push_back(this);
}
std::vector<SubscriptionLogger*> SubscriptionLogger::get_list_of_loggers()
{
    return loggerList;
}

std::vector<PublisherLogger*> PublisherLogger::get_list_of_loggers()
{
    return loggerList;
}


SubscriptionLogger::~SubscriptionLogger()
{
    auto vec = SubscriptionLogger::get_list_of_loggers();
    vec.erase(std::remove(vec.begin(), vec.end(), this), vec.end());
}
PublisherLogger::~PublisherLogger()
{
    auto vec = PublisherLogger::get_list_of_loggers();
    vec.erase(std::remove(vec.begin(), vec.end(), this), vec.end());
}


PublisherLogger::PublisherLogger(std::string topic_name)
{
    topic = topic_name;
    sent_counter = 0;
    loggerList.push_back(this);
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
uint32_t SubscriptionLogger::get_mean()
{
    uint32_t average = std::accumulate(time_diffs.begin(), time_diffs.end(), 0.0) / time_diffs.size();
    return average;
}

// https://stackoverflow.com/questions/9874802/how-can-i-get-the-maximum-or-minimum-value-in-a-vector
uint32_t SubscriptionLogger::get_max_latency()
{
    double max_latency = *std::max_element(time_diffs.begin(), time_diffs.end());
    return max_latency;
}

uint32_t SubscriptionLogger::get_min_latency()
{
    double min_latency = *std::min_element(time_diffs.begin(), time_diffs.end());
    return min_latency;
}
/*
void SubscriptionLogger::save_logged_data(std::string filename)
{
    
    std::ofstream RecordedFile;
    RecordedFile.open("~/dev_ws/src/loading_motor_dt/recorded_data/"+filename);
    RecordedFile << "topic,received(#)\n";
    auto loggers = SubscriptionLogger::get_list_of_loggers();
    for (auto& logger : loggers) {
        RecordedFile << logger->topic << "," << logger->recv_counter << "\n";
    }
    RecordedFile.close();
}
void PublisherLogger::save_logged_data(std::string filename)
{
    
    std::ofstream RecordedFile;
    RecordedFile.open("~/dev_ws/src/loading_motor_dt/recorded_data/"+filename);
    RecordedFile << "topic,sent(#)\n";
    auto loggers = PublisherLogger::get_list_of_loggers();
    for (auto& logger : loggers) {
        RecordedFile << logger->topic << "," << logger->sent_counter << "\n";
    }
    RecordedFile.close();
}*/

void DataLogger::save_logged_data(std::string filename)
{
    std::ofstream fRecordedSubs;
    std::ofstream fRecordedPubs;
    // "~/dev_ws/src/loading_motor_dt/recorded_data/subscribers/"
    fRecordedSubs.open("data_logger/subscriptions/"+filename);
    
    if(!fRecordedSubs.is_open())
        throw std::runtime_error("Could not open file subs:");
    
    fRecordedSubs << "topic,received(#)\n";
    auto loggers_sub = SubscriptionLogger::get_list_of_loggers();
    for (auto& logger : loggers_sub) {
        fRecordedSubs << logger->topic << "," << logger->recv_counter << "\n";
    }
    fRecordedSubs.close();

    fRecordedPubs.open("data_logger/publishers/"+filename);
    
    if(!fRecordedPubs.is_open())
        throw std::runtime_error("Could not open file pubs:");

    fRecordedPubs << "topic,sent(#)\n";
    auto loggers_pub = PublisherLogger::get_list_of_loggers();
    for (auto& logger : loggers_pub) {
        fRecordedPubs << logger->topic << "," << logger->sent_counter << "\n";
    }
    fRecordedPubs.close();

}