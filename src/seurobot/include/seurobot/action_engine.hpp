#pragma once

#include <seurobot/seu_robot.hpp>
#include <seumath/math.hpp>
#include <common/msg/body_angles.hpp>

namespace seurobot
{
class ActionEngine
{
public:
    ActionEngine(std::string act_file, std::shared_ptr<SeuRobot> robot);
    std::vector<common::msg::BodyAngles> runAction(std::string act, int idx=100);
    std::vector<std::string> getActions();

    ActMap& get_act_map()
    {
        return act_map_;
    }

    PosMap& get_pos_map()
    {
        return pos_map_;
    }

private:
    std::vector< std::map<RobotMotion, RobotPose> >
    get_poses(std::map<RobotMotion, RobotPose> &pos1,
              std::map<RobotMotion, RobotPose> &pos2, int act_time);

    bool get_degs(PoseMap &act_pose, common::msg::BodyAngles &bAngles);

private:
    ActMap act_map_;
    PosMap pos_map_;
    std::shared_ptr<SeuRobot> robot_;
};
}

