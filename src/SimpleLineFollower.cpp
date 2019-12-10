//
// Created by ros-aadc on 10.12.19.
//

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>
#include <ros_adas2019_examples/SimpleLineFollowerConfig.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

ros_adas2019_examples::SimpleLineFollowerConfig config = {};

void callback(ros_adas2019_examples::SimpleLineFollowerConfig &new_config, uint32_t level) {

    ROS_DEBUG("SimpleLineFollowerConfig: \tstartX: %f |\tendX: %f |\tsearchY: %f |\tdebugImageScaling: %f",
              new_config.startX, new_config.endX, new_config.searchY, new_config.debugImageScaling);

    config = new_config;
}


class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    ros::Publisher steeringPublisher;

public:
    ImageConverter()
            : it_(nh_)
    {



        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/pylon_camera_node/image_rect", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/simplelinefollower/debug_image", 1);

        steeringPublisher = nh_.advertise<std_msgs::Float32>("/adas2019/actuator/steering", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat grey = cv::Mat();
        cv::cvtColor(cv_ptr->image, grey, CV_BGR2GRAY);
        cv::Mat binary = cv::Mat();
        cv::adaptiveThreshold(grey, binary, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 101, -50.0);

        cv::Mat debugImage = cv::Mat();
        cv::cvtColor(binary, debugImage, CV_GRAY2BGR);

        static int startX, endX, searchY;
        startX  = (int) (config.startX     * binary.cols);
        endX    = (int) (config.endX       * binary.cols);
        searchY = (int) (config.searchY    * binary.rows);


        cv::line(debugImage, cv::Point2d(startX, searchY), cv::Point2d(endX, searchY), cv::Scalar(0,255,0));

        int x = startX;
        static int found_X = x;
        for ( ; x < endX; ++x) {
            cv::Point2d location = cv::Point2d(x, searchY);
            if (binary.at<uchar>(location) > 0) {
                found_X = x;
                break;
            }
        }

        cv::circle(debugImage, cv::Point2d(found_X, searchY), 5, cv::Scalar(0, 0, 255), -1);
        static std_msgs::Float32 steeringMsg;
        steeringMsg.data = ((float) (found_X - startX) / (float) (endX - startX)) * 200.0f - 100.0f;
        steeringPublisher.publish(steeringMsg);

        cv::resize(debugImage, debugImage, cv::Size(), config.debugImageScaling, config.debugImageScaling);

        auto out = cv_bridge::CvImage(cv_ptr->header, sensor_msgs::image_encodings::BGR8, debugImage);
        // Output modified video stream
        image_pub_.publish(out.toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;


    dynamic_reconfigure::Server<ros_adas2019_examples::SimpleLineFollowerConfig> server;
    server.setCallback(&callback);

    ros::spin();
    return 0;
}
