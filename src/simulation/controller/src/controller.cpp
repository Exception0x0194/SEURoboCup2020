#include <common/msg/body_angles.hpp>
#include <common/msg/head_angles.hpp>
#include <common/srv/get_angles.hpp>
#include <common/common.hpp>
#include <seumath/math.hpp>

#include "SimRobot.hpp"
#include <unistd.h>


using namespace std;
using namespace common;
using namespace seumath;
using namespace Eigen;

int main(int argc, char **argv)
{
    string robotName = "maxwell";
    if (argc > 1) {
        robotName = std::string(argv[1]);
    }
    setenv("WEBOTS_ROBOT_NAME", robotName.c_str(), 0);
    rclcpp::init(argc, argv);
    
    bool ok=false;
    int i=0;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(robotName + "_controller");
    while(rclcpp::ok() && i++<20) {
        FILE   *stream;
        char   buf[1024];
        memset( buf, '\0', sizeof(buf) );//初始化buf
        stream = popen( "ls /tmp | grep webots-", "r" );
        fread( buf, sizeof(char), sizeof(buf),  stream);  //将刚刚FILE* stream的数据流读取到buf中
        pclose( stream );
        string sbuf(buf);
        if(sbuf.find("webots") != string::npos) {
            string pf = "/tmp/"+sbuf.substr(0, sbuf.find("\n"))+"/WEBOTS_SERVER";
            RCLCPP_WARN(node->get_logger(), "%s", pf.c_str());
            if(access(pf.c_str(), F_OK)!=-1) {
                ok = true;
                break;
            }
        }
        RCLCPP_WARN(node->get_logger(), "waiting for webots ......");
        usleep(1000000);
    }
    if(!ok) return 0;

    std::shared_ptr<SimRobot> player = std::make_shared<SimRobot>(robotName);

    rclcpp::Client<common::srv::GetAngles>::SharedPtr client =
        node->create_client<common::srv::GetAngles>(robotName + "/get_angles");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }
    int ret = 0;
    while (rclcpp::ok() && ret >= 0) {
        auto request = std::make_shared<common::srv::GetAngles::Request>();
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            player->mAngles = result.get()->body;
            player->mHAngles = result.get()->head;
        }
        ret = player->myStep();

    }
    return 0;
}

