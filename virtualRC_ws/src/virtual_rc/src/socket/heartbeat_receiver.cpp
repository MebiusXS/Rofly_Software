#include "socket/heartbeat_receiver.h"

HeartbeatReceiver::HeartbeatReceiver(const int port, const double land_timeout, const double hover_timeout) : port_(port), land_timeout_(land_timeout), hover_timeout_(hover_timeout)
{
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        std::cerr << "Socket creation failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags < 0)
    {
        std::cerr << "Error getting socket flags" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        std::cerr << "Error setting socket to non-blocking" << std::endl;
        exit(EXIT_FAILURE);
    }

    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_addr.s_addr = INADDR_ANY;
    servaddr_.sin_port = htons(port_);

    if (bind(sockfd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0)
    {
        std::cerr << "Bind failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    len_ = sizeof(cliaddr_);
    last_heartbeat_time_ = ros::Time::now();
}

HeartbeatReceiver::~HeartbeatReceiver()
{
    close(sockfd_);
}

void HeartbeatReceiver::listenForHeartbeats()
{
    n_ = recvfrom(sockfd_, buffer_, sizeof(buffer_), 0, (struct sockaddr *)&cliaddr_, &len_);
    if (n_ < 0)
    {
        if (errno != EWOULDBLOCK)
        {
            std::cerr << "Heartbeat receive error" << std::endl;
        }
    }
    else
    {
        buffer_[n_] = '\0';
        std::cout << "[" << ros::Time::now() << "] " << "Heartbeat received: " << buffer_ << std::endl;

        last_heartbeat_time_ = ros::Time::now();
    }
}

bool HeartbeatReceiver::shouldLand()
{
    return (ros::Time::now() - last_heartbeat_time_).toSec() > land_timeout_;
}

bool HeartbeatReceiver::shouldHover()
{
    return (ros::Time::now() - last_heartbeat_time_).toSec() > hover_timeout_;
}
