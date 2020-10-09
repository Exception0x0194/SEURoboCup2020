#ifndef __UNIROBOT_HPP
#define __UNIROBOT_HPP

#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>

#include <common/msg/body_angles.hpp>
#include <common/msg/head_angles.hpp>
#include <common/msg/led_task.hpp>
#include <common/msg/imu_data.hpp>
#include <common/msg/head_angles.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <rclcpp/rclcpp.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher(std::string robot_name): Node(robot_name + "_image_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(robot_name + "/sensor/image", 5);
    }

    void Publish(const unsigned char* data, int w, int h)
    {
        auto message = sensor_msgs::msg::Image();
        message.header.stamp = rclcpp::Time();
        message.header.frame_id = "simulation";
        message.width = w;
        message.height = h;
        message.step = w * 3;
        message.encoding = sensor_msgs::image_encodings::RGB8;
        message.data.resize(w * h * 3);
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                message.data[i * w * 3 + j * 3 + 0] = *(data + i * w * 4 + j * 4 + 2);
                message.data[i * w * 3 + j * 3 + 1] = *(data + i * w * 4 + j * 4 + 1);
                message.data[i * w * 3 + j * 3 + 2] = *(data + i * w * 4 + j * 4 + 0);
            }
        }
        publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

class ImuPublisher : public rclcpp::Node
{
public:
    ImuPublisher(std::string robot_name): Node(robot_name + "_imu_publisher")
    {
        publisher_ = this->create_publisher<common::msg::ImuData>(robot_name + "/sensor/imu", 5);
    }

    void Publish(const common::msg::ImuData& data)
    {
        publisher_->publish(data);
    }

    rclcpp::Publisher<common::msg::ImuData>::SharedPtr publisher_;
};

class HeadAnglePublisher : public rclcpp::Node
{
public:
    HeadAnglePublisher(std::string robot_name): Node(robot_name + "_head_angle_publisher")
    {
        publisher_ = this->create_publisher<common::msg::HeadAngles>(robot_name + "/sensor/joint/head", 5);
    }

    void Publish(const common::msg::HeadAngles& data)
    {
        publisher_->publish(data);
    }

    rclcpp::Publisher<common::msg::HeadAngles>::SharedPtr publisher_;
};

class SimRobot: public webots::Robot
{
public:
    SimRobot(std::string robot_name="maxwell");
    int myStep();

    common::msg::BodyAngles mAngles;
    common::msg::HeadAngles mHAngles;

private:
    int mTimeStep;
    int totalTime;

    std::shared_ptr<ImagePublisher> mImagePublisher;
    std::shared_ptr<ImuPublisher> mImuPublisher;
    std::shared_ptr<HeadAnglePublisher> mHeadPublisher;

    void setPositions();
    void checkFall();

    void wait(int ms);

    webots::Camera *mCamera;
    webots::InertialUnit *mIMU;
    std::vector<webots::LED*> mLEDs;

    common::msg::LedTask ledTask;
    int fallType;
    bool imuReset;
    double mInitYaw;
    std::string robotName;
};


#endif
