#include "services.hpp"
#include <deque>
#include <mutex>

std::mutex bodyDequeMtx, headDequeMtx;
std::deque<common::msg::BodyAngles> bodyAngles;
std::deque<common::msg::HeadAngles> headAngles;

std::shared_ptr<seurobot::SeuRobot> maxwell;

std::shared_ptr<seurobot::SeuRobot> CreateRobot(std::string robotFile, std::string offsetFile)
{
    maxwell = std::make_shared<seurobot::SeuRobot>(robotFile, offsetFile);
    return maxwell;
}

void AddBodyAngles(const std::vector<common::msg::BodyAngles> &angles)
{
    bodyDequeMtx.lock();
    bodyAngles.insert(bodyAngles.end(), angles.begin(), angles.end());
    bodyDequeMtx.unlock();
}

int GetBodyAnglesSize()
{
    return bodyAngles.size();
}

void AddHeadAngles(const common::msg::HeadAngles &angles)
{
    headDequeMtx.lock();
    headAngles.push_back(angles);
    headDequeMtx.unlock();
}

void AddHeadAngles(const std::vector<common::msg::HeadAngles> &angles)
{
    headDequeMtx.lock();
    headAngles.insert(headAngles.end(), angles.begin(), angles.end());
    headDequeMtx.unlock();
}

int GetHeadAnglesSize()
{
    return headAngles.size();
}

void GetAnglesService(const std::shared_ptr<common::srv::GetAngles::Request> req,
                      std::shared_ptr<common::srv::GetAngles::Response> res)
{
    bodyDequeMtx.lock();
    if (!bodyAngles.empty()) {
        maxwell->set_body(bodyAngles.front());
        bodyAngles.pop_front();
    }
    bodyDequeMtx.unlock();
    headDequeMtx.lock();
    if (!headAngles.empty()) {
        maxwell->set_head(headAngles.front());
        headAngles.pop_front();
    }
    headDequeMtx.unlock();
    if(req->player == "real") {
        int sid = maxwell->joint_start_id();
        res->start_id = sid;
        for(int i=0; i<maxwell->joint_count(); i++) {
            auto joint = maxwell->get_joint(sid+i);
            float deg = (joint->current_deg + joint->offset) * joint->inverse;
            res->degs.push_back(deg);
        }
    } else {
        res->head.yaw = maxwell->get_joint("jhead1")->current_deg;
        res->head.pitch = maxwell->get_joint("jhead2")->current_deg;
        res->body.left_shoulder = maxwell->get_joint("jlshoulder1")->current_deg;
        res->body.left_elbow = maxwell->get_joint("jlelbow")->current_deg;
        res->body.right_shoulder = maxwell->get_joint("jrshoulder1")->current_deg;
        res->body.right_elbow = maxwell->get_joint("jrelbow")->current_deg;

        res->body.left_hip_yaw = maxwell->get_joint("jlhip3")->current_deg;
        res->body.left_hip_roll = maxwell->get_joint("jlhip2")->current_deg;
        res->body.left_hip_pitch = maxwell->get_joint("jlhip1")->current_deg;
        res->body.left_knee = maxwell->get_joint("jlknee")->current_deg;
        res->body.left_ankle_pitch = maxwell->get_joint("jlankle2")->current_deg;
        res->body.left_ankle_roll = maxwell->get_joint("jlankle1")->current_deg;

        res->body.right_hip_yaw = maxwell->get_joint("jrhip3")->current_deg;
        res->body.right_hip_roll = maxwell->get_joint("jrhip2")->current_deg;
        res->body.right_hip_pitch = maxwell->get_joint("jrhip1")->current_deg;
        res->body.right_knee = maxwell->get_joint("jrknee")->current_deg;
        res->body.right_ankle_pitch = maxwell->get_joint("jrankle2")->current_deg;
        res->body.right_ankle_roll = maxwell->get_joint("jrankle1")->current_deg;
    }
}

void AddAnglesService(const std::shared_ptr<common::srv::AddAngles::Request> req,
    std::shared_ptr<common::srv::AddAngles::Response> res)
{
    AddBodyAngles(req->angles);
    res->success = true;
}