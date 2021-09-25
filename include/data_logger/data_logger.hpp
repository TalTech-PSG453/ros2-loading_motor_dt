#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <iostream> // header in standard library
#include <vector>
#include <string>
#include <numeric>
#include <algorithm>
#include <fstream>

namespace DataLogger
{
    class SubscriptionLogger
    {
        public:

        /*variables*/

        std::string topic;
        std::vector<uint32_t> time_diffs;
        unsigned int recv_counter;
        /*function declarations*/
        
        SubscriptionLogger(std::string topic_name);
        /*
        float get_late(float period);
        float get_too_late(float period);
        */
        uint32_t get_mean();
        uint32_t get_max_latency();
        uint32_t get_min_latency();

        int save_logged_data(std::string filename);
    };

    class PublisherLogger
    {
        public:
        
        PublisherLogger(std::string topic_name);
        unsigned int sent_counter;
        std::string topic;
        int save_logged_data(std::string filename);
    };

    // double get_period(float frequency);
}
#endif