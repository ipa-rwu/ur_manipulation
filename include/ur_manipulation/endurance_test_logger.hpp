#ifndef ENDURANCE_TEST_LOGGER_HPP
#define ENDURANCE_TEST_LOGGER_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <chrono>
#include <ctime>


class EnduranceTestLogger
{
public:
    EnduranceTestLogger(std::string server_ip, unsigned short port);

    bool connect_to_server();
    std::string get_now_as_string();
    void send_data(std::string data, bool verbose=true, double sleep_duration=0);
    void send_data();
    void send_data_timestamped(std::string data, bool verbose=true, double sleep_duration=0);
    void send_data_timestamped();
    void setMessage(const std::string &message);
    void setVerbose(bool verbose);
    void setSleep_duration(double sleep_duration);

private:
    int _socket;
    int _valread;
    std::string _server_ip;
    std::string _message;
    bool _verbose;
    double _sleep_duration;
    unsigned short _port;
    struct sockaddr_in _serv_addr;
    char _buffer[1024] = {0};

    bool initialize_socket();
    bool check_and_assign_ip();
    void sleepSafeFor(double duration);
};

#endif // ENDURANCE_TEST_LOGGER_HPP
