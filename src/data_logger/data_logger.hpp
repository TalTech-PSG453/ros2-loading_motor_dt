namespace DataLogger
{
    class SubscriptionLogger
    {
        public:

        /*variables*/

        std::string topic;
        std::vector<double> time_diffs;
        unsigned int recv_counter;
        /*function declarations*/
        
        SubscriptionLogger();
        /*
        float get_late(float period);
        float get_too_late(float period);
        */
        double get_mean();
        double get_max_latency();
        double get_min_latency();
    };

    struct PublisherLogger
    {
        unsigned int sent_counter;
    };

    int save_logged_data(int mode, std::string filename);
    double get_period(float frequency);

}