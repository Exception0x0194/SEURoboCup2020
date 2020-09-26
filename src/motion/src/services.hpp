#pragma once

#include <rclcpp/rclcpp.hpp>
#include <common/msg/body_angles.hpp>
#include <common/msg/head_angles.hpp>
#include <common/srv/get_angles.hpp>
#include <common/srv/add_angles.hpp>
#include <seurobot/seu_robot.hpp>

std::shared_ptr<seurobot::SeuRobot> CreateRobot(std::string robotFile, std::string offsetFile="");

void GetAnglesService(const std::shared_ptr<common::srv::GetAngles::Request> req,
    std::shared_ptr<common::srv::GetAngles::Response> res);

void AddAnglesService(const std::shared_ptr<common::srv::AddAngles::Request> req,
    std::shared_ptr<common::srv::AddAngles::Response> res);

void AddBodyAngles(const std::vector<common::msg::BodyAngles> &angles);
int GetBodyAnglesSize();

void AddHeadAngles(const common::msg::HeadAngles &angles);
void AddHeadAngles(const std::vector<common::msg::HeadAngles> &angles);
int GetHeadAnglesSize();