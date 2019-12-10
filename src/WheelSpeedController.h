//
// Created by ros-aadc on 10.12.19.
//

#ifndef ROS_ADAS2019_EXAMPLES_WHEELSPEEDCONTROLLER_H
#define ROS_ADAS2019_EXAMPLES_WHEELSPEEDCONTROLLER_H


class WheelSpeedController {

public:
    WheelSpeedController(ros::NodeHandle &nh);
    void updateController();

private:

    ros::Subscriber odometrySpeedSubscriber;
    ros::Subscriber targetSpeedSubscriber;
    ros::Subscriber emergencyBrakeSubscriber;

    ros::Publisher actuatorSpeedPublisher;

    bool emergencyBrakeIsActive = false;

    double currentSpeed;
    double desiredSpeed;

    double accumulatedError = 0;
    ros::Time lastUpdateTime;

    void updateCurrentSpeed(const std_msgs::Float32::ConstPtr &msg);
    void updateDesiredSpeed(const std_msgs::Float32::ConstPtr &msg);
    void updateEmergencyBrake(const std_msgs::Bool::ConstPtr &msg);


    void reset();
};


#endif //ROS_ADAS2019_EXAMPLES_WHEELSPEEDCONTROLLER_H
