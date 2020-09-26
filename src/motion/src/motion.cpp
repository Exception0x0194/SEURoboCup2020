#include <seurobot/action_engine.hpp>
#include <seurobot/seu_robot.hpp>
#include <seumath/math.hpp>

#include "subscribers.hpp"
#include "services.hpp"
#include "walk/WalkEngine.hpp"

using namespace Eigen;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string robotName = "maxwell";
    if (argc > 1) {
        robotName = std::string(argv[1]);
    }

    auto node = std::make_shared<rclcpp::Node>(robotName + "_motion");
    auto paramClient = std::make_shared<rclcpp::SyncParametersClient>(node, "parameter");

    while (!paramClient->wait_for_service()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }
    auto parameters = paramClient->get_parameters({"robot_file", "offset_file", "action_file", "walk_file"});
    std::string act_file, robot_file, offset_file, walk_file;
    robot_file = parameters[0].value_to_string();
    offset_file = parameters[1].value_to_string();
    act_file = parameters[2].value_to_string();
    walk_file = parameters[3].value_to_string();

    auto robot = CreateRobot(robot_file, offset_file);
    auto actEng = std::make_shared<seurobot::ActionEngine>(act_file, robot);
    auto walkEng = std::make_shared<WalkEngine>(walk_file, robot);
    auto bodyTaskSubscriber = std::make_shared<BodyTaskSubSubscriber>(robotName);
    auto headTaskSubscriber = std::make_shared<HeadTaskSubSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto gameSubscriber = std::make_shared<GameDataSubscriber>(robotName);

    auto getAnglesNode = rclcpp::Node::make_shared(robotName + "_GetAnglesSrv");
    auto getAnglesSrv = getAnglesNode->create_service<common::srv::GetAngles>(robotName + "/get_angles", &GetAnglesService);
    auto addAnglesNode = rclcpp::Node::make_shared(robotName + "_AddAnglesSrv");
    auto addAnglesSrv = addAnglesNode->create_service<common::srv::AddAngles>(robotName + "/add_angles", &AddAnglesService);

    double phase = 0.0;
    bool isWalking = false;
    std::vector<common::msg::BodyAngles> angles = actEng->runAction("reset");
    AddBodyAngles(angles);

    while (rclcpp::ok()) {
        rclcpp::spin_some(bodyTaskSubscriber);
        rclcpp::spin_some(headTaskSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(gameSubscriber);
        rclcpp::spin_some(getAnglesNode);
        rclcpp::spin_some(addAnglesNode);
        rclcpp::spin_some(node);

        if (GetHeadAnglesSize() < 5) {
            auto task = headTaskSubscriber->GetTask();
            common::msg::HeadAngles angle;
            angle.pitch = task.pitch;
            angle.yaw = task.yaw;
            AddHeadAngles(angle);
        }

        if (GetBodyAnglesSize() < 5) {
            auto task = bodyTaskSubscriber->GetTask();
            auto imu = imuSubscriber->GetData();
            auto game = gameSubscriber->GetData();
            angles.clear();
            if (imu.fall == imu.FALL_FORWARD) {
                angles = actEng->runAction("front_getup");
            } else if (imu.fall == imu.FALL_BACKWARD) {
                angles = actEng->runAction("back_getup");
            } else if (imu.fall == imu.FALL_LEFT) {
                angles = actEng->runAction("left_getup");
            } else if (imu.fall == imu.FALL_RIGHT) {
                angles = actEng->runAction("right_getup");
            } else {
                if (game.state == game.STATE_INIT || game.state == game.STATE_PAUSE) {
                    angles = actEng->runAction("reset");
                } else if (game.state == game.STATE_PLAY) {
                    bool isNormal = true;
                    for (size_t i = 0; i < 2; i++) {
                        if (game.red_players[i].name == robotName && game.red_players[i].state != common::msg::Player::PLAYER_NORMAL) {
                            isNormal = false;
                            break;
                        }
                        if (game.blue_players[i].name == robotName && game.blue_players[i].state != common::msg::Player::PLAYER_NORMAL) {
                            isNormal = false;
                            break;
                        }
                    }
                    if (!isNormal) {
                        angles = actEng->runAction("reset");
                    } else {
                        if (task.type == common::msg::BodyTask::TASK_WALK) {
                            if (task.count > 0) {
                                if (!isWalking) {
                                    angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 2, phase);
                                    isWalking = true;
                                } else {
                                    angles = walkEng->runWalk(Vector3d(task.step, task.lateral, task.turn), task.count, phase);
                                }
                            } else {
                                if (isWalking) {
                                    angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 0, phase);
                                    isWalking = false;
                                }
                            }
                        } else if (task.type == common::msg::BodyTask::TASK_ACT) {
                            if (task.count >= 0) {
                                if (isWalking) {
                                    angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 0, phase);
                                    isWalking = false;
                                }
                                angles = actEng->runAction(task.actname);
                            }
                        } else {
                            if (isWalking) {
                                angles = walkEng->runWalk(Vector3d(0.0, 0.0, 0.0), 0, phase);
                                isWalking = false;
                            }
                        }
                    }
                } else {
                    angles = actEng->runAction("ready");
                }
            }
            AddBodyAngles(angles);
        }
    }
    return 0;
}
