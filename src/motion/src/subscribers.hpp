#pragma once

#include <common/msg/body_task.hpp>
#include <common/msg/head_task.hpp>
#include <common/msg/imu_data.hpp>
#include <common/msg/game_data.hpp>
#include <rclcpp/rclcpp.hpp>

class BodyTaskSubSubscriber: public rclcpp::Node
{
public:
    BodyTaskSubSubscriber(std::string robotName): Node(robotName + "_body_task_subscriber")
    {
        subscription_ = this->create_subscription<common::msg::BodyTask>(
            robotName + "/task/body", 5, std::bind(&BodyTaskSubSubscriber::topic_callback, this,
            std::placeholders::_1));
    }

    const common::msg::BodyTask& GetTask()
    {
        return bodyTask_;
    }
    
private:
    void topic_callback(const common::msg::BodyTask::SharedPtr msg)
    {
        bodyTask_ = *msg;
    }

    rclcpp::Subscription<common::msg::BodyTask>::SharedPtr subscription_;
    common::msg::BodyTask bodyTask_;
};

class HeadTaskSubSubscriber: public rclcpp::Node
{
public:
    HeadTaskSubSubscriber(std::string robotName): Node(robotName + "_head_task_subscriber")
    {
        subscription_ = this->create_subscription<common::msg::HeadTask>(
            robotName + "/task/head", 5, std::bind(&HeadTaskSubSubscriber::topic_callback, this,
            std::placeholders::_1));
    }

    const common::msg::HeadTask& GetTask()
    {
        return headTask_;
    }

private:
    void topic_callback(const common::msg::HeadTask::SharedPtr msg)
    {
        headTask_ = *msg;
    }

    common::msg::HeadTask headTask_;
    rclcpp::Subscription<common::msg::HeadTask>::SharedPtr subscription_;
};

class ImuDataSubscriber: public rclcpp::Node
{
public:
    ImuDataSubscriber(std::string robotName): Node(robotName + "_imu_data_subscriber_motion")
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

class GameDataSubscriber: public rclcpp::Node
{
public:
    GameDataSubscriber(std::string robotName): Node(robotName + "_game_data_subscriber")
    {
        subscription_ = this->create_subscription<common::msg::GameData>(
            "/sensor/game", 5, std::bind(&GameDataSubscriber::topic_callback, this,
            std::placeholders::_1));
    }

    const common::msg::GameData& GetData()
    {
        return gameData_;
    }

private:
    void topic_callback(const common::msg::GameData::SharedPtr msg)
    {
        gameData_ = *msg;
    }

    common::msg::GameData gameData_;
    rclcpp::Subscription<common::msg::GameData>::SharedPtr subscription_;
};