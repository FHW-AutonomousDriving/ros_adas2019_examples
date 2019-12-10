//
// Created by ros-aadc on 10.12.19.
//

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <ros_adas2019_examples/WheelSpeedControllerConfig.h>

#include "WheelSpeedController.h"

#define max(x, y) (x > y ? x : y)
#define min(x, y) (x < y ? x : y)
#define constrain(x, l, u) (min(max(x, l), u))
#define constrainPower(x) constrain(x, -config.maximumOutputPower, config.maximumOutputPower)


ros_adas2019_examples::WheelSpeedControllerConfig config = {};

void callback(ros_adas2019_examples::WheelSpeedControllerConfig &new_config, uint32_t level) {

    ROS_DEBUG("WheelSpeedControllerConfig: \tkP: %f |\tkI: %f |\tkD: %f |\tmaximumOutputPower: %f",
            new_config.kP, new_config.kI, new_config.kD, config.maximumOutputPower);

    config = new_config;
}


int main(int argc, char **argv) {


    ros::init(argc, argv, "WheelSpeedControllerNode");
    ros::NodeHandle nh;
    WheelSpeedController node(nh);


    dynamic_reconfigure::Server<ros_adas2019_examples::WheelSpeedControllerConfig> server;
    server.setCallback(&callback);

    ros::Rate update_rate(120);

    while(ros::ok()) {
        ros::spinOnce();

        node.updateController();
        update_rate.sleep();
    }

    return 0;
}



WheelSpeedController::WheelSpeedController(ros::NodeHandle &nh) {
    odometrySpeedSubscriber = nh.subscribe<std_msgs::Float32>("odometry/speed", 1, &WheelSpeedController::updateCurrentSpeed, this);
    targetSpeedSubscriber = nh.subscribe<std_msgs::Float32>("desired/speed", 1, &WheelSpeedController::updateDesiredSpeed, this);
    emergencyBrakeSubscriber = nh.subscribe<std_msgs::Bool>("desired/emergency_brake", 1, &WheelSpeedController::updateEmergencyBrake, this);
    actuatorSpeedPublisher = nh.advertise<std_msgs::Float32>("actuator/speed", 1);

    lastUpdateTime = ros::Time::now();
}

void WheelSpeedController::updateCurrentSpeed(const std_msgs::Float32::ConstPtr &msg) {
    currentSpeed = msg->data;
}

void WheelSpeedController::updateDesiredSpeed(const std_msgs::Float32::ConstPtr &msg) {
    desiredSpeed = msg->data;
}

void WheelSpeedController::updateEmergencyBrake(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
        // if the request is sent the first time and the integral part isn't already contributing to 'breaking' -> reset controller
//        bool accumulatorIsCounterProductive = (currentSpeed > 0 && accumulatedError > 0) || (currentSpeed < 0 && accumulatedError < 0);
//        if (!emergencyBrakeIsActive && accumulatorIsCounterProductive) reset();
//        if (!emergencyBrakeIsActive) reset();

        emergencyBrakeIsActive = true;
    } else {
        emergencyBrakeIsActive = false;
    }


}

void WheelSpeedController::updateController() {
    static std_msgs::Float32 message;

    ros::Time now = ros::Time::now();
    double deltaT = (now - lastUpdateTime).toSec();
    double error = (emergencyBrakeIsActive ? 0 : desiredSpeed) - currentSpeed;

    // calculate integral part and store it for later
    accumulatedError += constrainPower(config.kI * error * deltaT);

    // add all parts together (P, I and D)
    message.data = constrainPower(config.kP * error
                 + accumulatedError
                 + config.kD * error / deltaT);

    // Publish 0 if close to it to avoid beeping of motor
    if (abs(message.data) < config.minimumOutputPower) {
        message.data = 0;
    }
    actuatorSpeedPublisher.publish(message);

    // update for next loop
    lastUpdateTime = now;
}

void WheelSpeedController::reset() {
    accumulatedError = 0;
}
