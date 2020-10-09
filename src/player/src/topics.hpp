#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <common/msg/body_task.hpp>
#include <common/msg/head_task.hpp>
#include <common/msg/imu_data.hpp>
#include <common/msg/head_angles.hpp>
#include <opencv4/opencv2/opencv.hpp>

class BodyTaskPublisher : public rclcpp::Node
{
public:
    BodyTaskPublisher(std::string robot_name): Node(robot_name + "_body_task_publisher")
    {
        publisher_ = this->create_publisher<common::msg::BodyTask>(robot_name + "/task/body", 5);
    }

    void Publish(const common::msg::BodyTask& data)
    {
        publisher_->publish(data);
    }

    rclcpp::Publisher<common::msg::BodyTask>::SharedPtr publisher_;
};

class HeadTaskPublisher : public rclcpp::Node
{
public:
    HeadTaskPublisher(std::string robot_name): Node(robot_name + "_head_task_publisher")
    {
        publisher_ = this->create_publisher<common::msg::HeadTask>(robot_name + "/task/head", 5);
    }

    void Publish(const common::msg::HeadTask& data)
    {
        publisher_->publish(data);
    }

    rclcpp::Publisher<common::msg::HeadTask>::SharedPtr publisher_;
};

class ResultImagePublisher : public rclcpp::Node
{
public:
    ResultImagePublisher(std::string robot_name): Node(robot_name + "_result_image_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(robot_name + "/result/image", 5);
    }

    void Publish(cv::Mat &mat)
    {
        if (mat.empty()) {
            return;
        }
        auto message = sensor_msgs::msg::Image();
        message.header.stamp = rclcpp::Time();
        message.header.frame_id = "result";
        message.width = mat.cols;
        message.height = mat.rows;
        message.step = mat.cols * mat.channels();
        message.encoding = sensor_msgs::image_encodings::RGB8;
        message.data.resize(mat.cols * mat.rows * mat.channels());
        memcpy(&message.data[0], mat.data, mat.cols * mat.rows * mat.channels());
        publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

class ImuDataSubscriber: public rclcpp::Node
{
public:
    ImuDataSubscriber(std::string robotName): Node(robotName + "_imu_data_subscriber_player")
    {
        subscription_ = this->create_subscription<common::msg::ImuData>(
            robotName + "/sensor/imu", 5, std::bind(&ImuDataSubscriber::topic_callback, this,
            std::placeholders::_1));
    }

    const common::msg::ImuData& GetData()
    {
        return imuData_;
    }

private:
    void topic_callback(const common::msg::ImuData::SharedPtr msg)
    {
        imuData_ = *msg;
    }

    common::msg::ImuData imuData_;
    rclcpp::Subscription<common::msg::ImuData>::SharedPtr subscription_;
};

class ImageSubscriber: public rclcpp::Node
{
public:
    ImageSubscriber(std::string robotName): Node(robotName + "_image_data_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            robotName + "/sensor/image", 5, std::bind(&ImageSubscriber::topic_callback, this,
            std::placeholders::_1));
    }

    cv::Mat GetImage()
    {
        cv::Mat mat(image_.height, image_.width, CV_8UC3, &(image_.data[0]));
        return mat;
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_ = *msg;
    }

    sensor_msgs::msg::Image image_;
    cv::Mat cvMat_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

class HeadAngleSubscriber: public rclcpp::Node
{
public:
    HeadAngleSubscriber(std::string robotName): Node(robotName + "_head_angle_subscriber")
    {
        subscription_ = this->create_subscription<common::msg::HeadAngles>(
            robotName + "/sensor/joint/head", 5, std::bind(&HeadAngleSubscriber::topic_callback, this,
            std::placeholders::_1));
    }

    const common::msg::HeadAngles& GetData()
    {
        return headAngles_;
    }

private:
    void topic_callback(const common::msg::HeadAngles::SharedPtr msg)
    {
        headAngles_ = *msg;
    }

    common::msg::HeadAngles headAngles_;
    rclcpp::Subscription<common::msg::HeadAngles>::SharedPtr subscription_;
};