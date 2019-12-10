// created by Franz Wernicke on 21. Jan 2020

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

class EmergencyBrake {
    public:
    EmergencyBrake(ros::NodeHandle &nh);

    private:
    ros::Subscriber lidarSubscriber;
    ros::Subscriber steeringSubscriber;
    ros::Publisher emergencyBrakePublisher;

    std_msgs::Bool emergency_brake_message;

    float lastSteering = 0.0;

    void onNewLidarData(const sensor_msgs::LaserScan::ConstPtr &msg);
    void onNewSteeringData(const std_msgs::Float32::ConstPtr &msg);

    bool checkEmergencyBrake(const sensor_msgs::LaserScan::ConstPtr &msg);
    void sendBrakeSignal(bool brake);
};

EmergencyBrake::EmergencyBrake(ros::NodeHandle &nh) {
    lidarSubscriber = nh.subscribe<sensor_msgs::LaserScan>("/lidar/scan", 1, &EmergencyBrake::onNewLidarData, this);
    steeringSubscriber = nh.subscribe<std_msgs::Float32>("actuator/steering", 1, &EmergencyBrake::onNewSteeringData, this);
    emergencyBrakePublisher = nh.advertise<std_msgs::Bool>("desired/emergency_brake", 1);
}

void EmergencyBrake::onNewLidarData(const sensor_msgs::LaserScan::ConstPtr &msg) {
    bool brake = checkEmergencyBrake(msg);
    sendBrakeSignal(brake);
}

void EmergencyBrake::onNewSteeringData(const std_msgs::Float32::ConstPtr &msg) {
    lastSteering = msg->data;
}

bool EmergencyBrake::checkEmergencyBrake(const sensor_msgs::LaserScan::ConstPtr &msg) {
    auto vector =  msg->ranges;
    for (int i = 0; i < vector.size(); ++i) {
        float sample = vector.at(i);

        float angle = i * msg->angle_increment;
        float steeringAngle = lastSteering * M_PI / (180 * 3);
        angle += steeringAngle;
        float distance = cos(angle)*sample;
//        float steeringCompensation = tan() * distance; // approximate the rad value of the steering angle (-100 - 100)
        float distFromCenter = sin(angle)*sample;// - steeringCompensation;

        //                       CAR_WIDTH   MIN_SIDE       MIN_FRONT
        if (std::abs(distFromCenter) < ((0.3)/2 + 0.05) && distance < 0.3) {
            // ROS_INFO("Compensation: %f\t Steering: %f", steeringCompensation, m_currentSteeringAngleValue.f32Value);
//            ROS_INFO("distFromCenter: %f\t Distance: %f", distFromCenter, distance);
            //LOG_ERROR("Collision at distance: %f with angle: %f", sample.f32Radius, angle);
            return true;
        }
    }

    return false;
}

void EmergencyBrake::sendBrakeSignal(bool brake) {
    emergency_brake_message.data = brake;
    emergencyBrakePublisher.publish(emergency_brake_message);
}

int main(int argc, char **argv) {
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "emergency_brake_node");
    ros::NodeHandle nh;
    EmergencyBrake node(nh);

    ros::spin();
}