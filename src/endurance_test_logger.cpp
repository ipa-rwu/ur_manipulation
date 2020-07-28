#include "ur_manipulation/endurance_test_logger.hpp"

EnduranceTestLogger::EnduranceTestLogger(std::string server_ip, unsigned short port):
    _socket(0),
    _valread(0),
    _server_ip(server_ip),
    _port(port)
{
    _serv_addr.sin_family = AF_INET;
    _serv_addr.sin_port = htons(port);
    initialize_socket();
    check_and_assign_ip();
}

bool EnduranceTestLogger::connect_to_server()
{
    if (connect(_socket,  (struct sockaddr *)&_serv_addr, sizeof(_serv_addr)) < 0)
    {
        ROS_ERROR_STREAM("Failed to connect to " << _server_ip << ":" << _port);
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Connected to " << _server_ip << ":" << _port);
        return true;
    }
}

void EnduranceTestLogger::send_data(std::string data, bool verbose, double sleep_duration)
{
    send(_socket , data.c_str() , data.length() , 0 );
    sleepSafeFor(sleep_duration);
    if (verbose)
    {
        ROS_INFO_STREAM("Sent " << data << " >> " << _server_ip << ":" << _port );
    }
}

void EnduranceTestLogger::send_data()
{
    send(_socket , _message.c_str() , _message.length() , 0 );
    sleepSafeFor(_sleep_duration);
    if (_verbose)
    {
        ROS_INFO_STREAM("Sent " << _message << " >> " << _server_ip << ":" << _port );
    }
}

void EnduranceTestLogger::send_data_timestamped(std::string data, bool verbose, double sleep_duration)
{
    send_data(get_now_as_string() + ": "+ data, verbose, sleep_duration);
}

void EnduranceTestLogger::send_data_timestamped()
{
    send_data(get_now_as_string() + ": "+ _message, _verbose, _sleep_duration);
}

std::string EnduranceTestLogger::get_now_as_string()
{
    std::time_t t = std::time(nullptr);   // get time now
    std::tm* now = std::localtime(&t);
    std::string time= std::to_string(now->tm_hour) + ":" +
            std::to_string(now->tm_min) + ":" +
            std::to_string(now->tm_sec);
    return time;
}

void EnduranceTestLogger::setMessage(const std::string &message)
{
    _message = message;
}

void EnduranceTestLogger::setVerbose(bool verbose)
{
    _verbose = verbose;
}

void EnduranceTestLogger::setSleep_duration(double sleep_duration)
{
    _sleep_duration = sleep_duration;
}

bool EnduranceTestLogger::initialize_socket()
{
    if ((_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_ERROR_STREAM("Socket creation error.");
        return false;
    }
    else
    {
        return true;
    }

}

bool EnduranceTestLogger::check_and_assign_ip()
{
    ROS_INFO_STREAM("Checking ip: " << _server_ip);
    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, _server_ip.c_str(), &_serv_addr.sin_addr)<=0)
    {
        ROS_ERROR_STREAM("Invalid address/Address not supported: " << _server_ip);
        return false;
    }
    else
    {
        return true;
    }
}

void EnduranceTestLogger::sleepSafeFor(double duration)
{
    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start <= ros::Duration(duration))
    {
      ros::spinOnce();
    }
}
