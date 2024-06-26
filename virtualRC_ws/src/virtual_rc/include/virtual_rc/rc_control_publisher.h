#ifndef RC_CONTROL_PUBLISHER_H_
#define RC_CONTROL_PUBLISHER_H_

#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>

enum class RC_MODE
{
    STABILIZE = 0,
    AIRGROUND = 1,
    AUTOMATION = 2
};

class RCControlPublisher
{
public:
    RCControlPublisher() = delete;
    RCControlPublisher(const ros::NodeHandle &nh);
    RCControlPublisher(const RCControlPublisher &rhs) = delete;
    RCControlPublisher &operator=(const RCControlPublisher &rhs) = delete;
    RCControlPublisher(RCControlPublisher &&rhs) = delete;
    RCControlPublisher &operator=(RCControlPublisher &&rhs) = delete;
    virtual ~RCControlPublisher() {}

    void rc_publish();
    void emergency_stop();
    void cancel_emergency_stop();

    void set_mode(RC_MODE mode);
    void set_channels(int roll, int pitch, int throttle, int yaw, RC_MODE mode, bool emergency);

private:
    ros::NodeHandle nh_;
    ros::Publisher rc_pub_;
    mavros_msgs::OverrideRCIn rc_message_;
};

#endif // RC_CONTROL_PUBLISHER_H_