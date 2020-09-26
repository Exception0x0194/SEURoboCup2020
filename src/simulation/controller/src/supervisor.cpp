#include <webots/Supervisor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/msg/game_data.hpp>
#include <common/msg/field_data.hpp>
#include <common/msg/player.hpp>
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include <cmath>

using namespace std;

struct ObjectInfo {
    Eigen::Vector3d translation;
    Eigen::Vector4d rotation;
};

const double ballR = 0.07;
const double robotR = 0.2;
const double fieldX = 4.5 + ballR;
const double fieldZ = 3.0 + ballR;

double Sign(double v)
{
    return v < 0 ? -1 : 1;
}

bool IsEqual(double v1, double v2)
{
    return fabs(v1 - v2) < 1E-2;
}

bool BallMoved(const double *p1, const double *p2)
{
    return !(IsEqual(p1[0], p2[0]) && IsEqual(p1[2], p2[2]));
}

bool IsOutOfField(const double *pos)
{
    if ((fabs(pos[0]) > fieldX + robotR) || (fabs(pos[2]) > fieldZ + robotR)) {
        return true;
    } else {
        return false;
    }
}

class FieldDataPublisher : public rclcpp::Node
{
public:
    FieldDataPublisher(): Node("field_data_publisher")
    {
        publisher_ = this->create_publisher<common::msg::FieldData>("/sensor/field", 5);
    }

    void Publish(const common::msg::FieldData& data)
    {
        publisher_->publish(data);
    }

    rclcpp::Publisher<common::msg::FieldData>::SharedPtr publisher_;
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

int main(int argc, char **argv)
{
    const int basicTime = 20;
    const double goalX = 4.5 + ballR;
    const double goalZ = 0.7 + ballR;
    const double ballPosInit[3] = {0.0, ballR, 0.0};
    const double zeroVel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ObjectInfo redInitInfos[] = {
        {Eigen::Vector3d(0.75, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)},
        {Eigen::Vector3d(4.0, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)}
    };
    ObjectInfo blueInitInfos[] = {
        {Eigen::Vector3d(-0.75, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, 0)},
        {Eigen::Vector3d(-4.0, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, 0)}
    };

    ObjectInfo redWaitInfos[] = {
        {Eigen::Vector3d(0.75, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)},
        {Eigen::Vector3d(0.75, 0.365, 3.0), Eigen::Vector4d(0, 1, 0, M_PI / 2)}
    };
    ObjectInfo blueWaitInfos[] = {
        {Eigen::Vector3d(-0.75, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)},
        {Eigen::Vector3d(-0.75, 0.365, 3.0), Eigen::Vector4d(0, 1, 0, M_PI / 2)}
    };

    setenv("WEBOTS_ROBOT_NAME", "judge", 0);
    rclcpp::init(argc, argv);
    auto judgeNode = make_shared<rclcpp::Node>("judge");
    auto fieldPublisher = make_shared<FieldDataPublisher>();
    auto gameSubscriber = make_shared<GameDataSubscriber>("judge");
    bool ok = false;
    int i = 0;

    while (rclcpp::ok() && i++ < 20) {
        FILE   *stream;
        char   buf[1024];
        memset(buf, '\0', sizeof(buf));  //初始化buf
        stream = popen("ls /tmp | grep webots-", "r");
        fread(buf, sizeof(char), sizeof(buf),  stream);   //将刚刚FILE* stream的数据流读取到buf中
        pclose(stream);
        string sbuf(buf);

        if (sbuf.find("webots") != string::npos) {
            string pf = "/tmp/" + sbuf.substr(0, sbuf.find("\n")) + "/WEBOTS_SERVER";
            RCLCPP_WARN(judgeNode->get_logger(), "%s", pf.c_str());

            if (access(pf.c_str(), F_OK) != -1) {
                ok = true;
                break;
            }
        }

        RCLCPP_WARN(judgeNode->get_logger(), "waiting for webots ......");
        usleep(1000000);
    }

    if (!ok) {
        return 0;
    }

    common::msg::GameData gameData;
    common::msg::FieldData fieldData;
    std::shared_ptr<webots::Supervisor> super = make_shared<webots::Supervisor>();
    webots::Node *ball = super->getFromDef("Ball");
    webots::Node *red_1 = super->getFromDef("red_1");
    webots::Node *red_2 = super->getFromDef("red_2");
    webots::Node *blue_1 = super->getFromDef("blue_1");
    webots::Node *blue_2 = super->getFromDef("blue_2");
    webots::Node *redPlayers[2] = {red_1, red_2};
    webots::Node *bluePlayers[2] = {blue_1, blue_2};
    int ballMoveCnt = 0;
    double lastBallPos[3] = {0};
    auto lastGoalTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    fieldData.red_players[0].name = "red_1";
    fieldData.red_players[1].name = "red_2";
    fieldData.blue_players[0].name = "blue_1";
    fieldData.blue_players[1].name = "blue_2";
    bool initSet = false;
    bool redWaitSet[2] = {false, false};
    bool blueWaitSet[2] = {false, false};

    while (super->step(basicTime) != -1 && rclcpp::ok()) {
        rclcpp::spin_some(judgeNode);
        rclcpp::spin_some(fieldPublisher);
        rclcpp::spin_some(gameSubscriber);
        gameData = gameSubscriber->GetData();
        if (gameData.state == gameData.STATE_INIT) {
            if (!initSet) {
                ball->setVelocity(zeroVel);
                ball->getField("translation")->setSFVec3f(ballPosInit);
                for (size_t i = 0; i < 2; i++) {
                    redPlayers[i]->getField("translation")->setSFVec3f(redInitInfos[i].translation.data());
                    redPlayers[i]->getField("rotation")->setSFRotation(redInitInfos[i].rotation.data());
                    redPlayers[i]->setVelocity(zeroVel);
                    bluePlayers[i]->getField("translation")->setSFVec3f(blueInitInfos[i].translation.data());
                    bluePlayers[i]->getField("rotation")->setSFRotation(blueInitInfos[i].rotation.data());
                    bluePlayers[i]->setVelocity(zeroVel);
                }
                initSet = true;
            }
        } else {
            initSet = false;
        }
        for (size_t i = 0; i < 2; i++) {
            if (gameData.red_players[i].state == common::msg::Player::PLAYER_WAIT) {
                if (!redWaitSet[i]) {
                    redWaitSet[i] = true;
                    redPlayers[i]->getField("translation")->setSFVec3f(redWaitInfos[i].translation.data());
                    redPlayers[i]->getField("rotation")->setSFRotation(redWaitInfos[i].rotation.data());
                    redPlayers[i]->setVelocity(zeroVel);
                }
            } else {
                redWaitSet[i] = false;
            }
            if (gameData.blue_players[i].state == common::msg::Player::PLAYER_WAIT) {
                if (!blueWaitSet[i]) {
                    blueWaitSet[i] = true;
                    bluePlayers[i]->getField("translation")->setSFVec3f(blueWaitInfos[i].translation.data());
                    bluePlayers[i]->getField("rotation")->setSFRotation(blueWaitInfos[i].rotation.data());
                    bluePlayers[i]->setVelocity(zeroVel);
                }
            } else {
                blueWaitSet[i] = false;
            }
        }

        fieldData.red_players[0].state = IsOutOfField(red_1->getPosition()) ?
                                         common::msg::Player::PALYER_OUT : common::msg::Player::PLAYER_NORMAL;
        fieldData.red_players[1].state = IsOutOfField(red_2->getPosition()) ?
                                         common::msg::Player::PALYER_OUT : common::msg::Player::PLAYER_NORMAL;
        fieldData.blue_players[0].state = IsOutOfField(blue_1->getPosition()) ?
                                          common::msg::Player::PALYER_OUT : common::msg::Player::PLAYER_NORMAL;
        fieldData.blue_players[1].state = IsOutOfField(blue_2->getPosition()) ?
                                          common::msg::Player::PALYER_OUT : common::msg::Player::PLAYER_NORMAL;

        const double *ballPos = ball->getPosition();
        bool goal = false;
        auto currTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        if (fabs(ballPos[2]) < goalZ) {
            if (ballPos[0] < -goalX) {
                if (currTime - lastGoalTime > 10) {
                    fieldData.red_score++;
                }
                goal = true;
            } else if (ballPos[0] > goalX) {
                if (currTime - lastGoalTime > 10) {
                    fieldData.blue_score++;
                }
                goal = true;
            }
        }
        if (goal) {
            lastGoalTime = currTime;
            fieldData.ball_state = fieldData.BALL_GOAL;
        } else {
            if (fabs(ballPos[2]) > fieldZ || fabs(ballPos[0]) > fieldX) {
                fieldData.ball_state = fieldData.BALL_OUT;
            } else {
                if (BallMoved(lastBallPos, ballPos)) {
                    ballMoveCnt = 0;
                    memcpy(lastBallPos, ballPos, 3 * sizeof(double));
                } else {
                    ballMoveCnt++;
                }

                if (ballMoveCnt * basicTime > 60 * 1000) {
                    fieldData.ball_state = fieldData.BALL_NOMOVE;
                } else {
                    fieldData.ball_state = fieldData.BALL_NORMAL;
                }
            }
        }
        fieldPublisher->Publish(fieldData);
    }

    return 0;
}
