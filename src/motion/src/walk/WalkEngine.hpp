#pragma once

#include <eigen3/Eigen/Dense>
#include <seurobot/seu_robot.hpp>
#include <common/msg/body_angles.hpp>
#include "IKWalk.hpp"

class WalkEngine
{
public:
    WalkEngine(std::string wkfile, std::shared_ptr<seurobot::SeuRobot> robot);
    std::vector<common::msg::BodyAngles> runWalk(Eigen::Vector3d p, int steps, double& phase);
    
private:
    double engine_frequency_;
    double time_length_;
    double XOffset_, YOffset_, DOffset_;
    Rhoban::IKWalkParameters params_;
    Eigen::Vector2d xrange_, yrange_, drange_;
    std::shared_ptr<seurobot::SeuRobot> robot_;
};
