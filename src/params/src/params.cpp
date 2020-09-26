#include <rclcpp/rclcpp.hpp>
#include <basic_parser/basic_parser.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

template<typename T>
void SetParameter(std::shared_ptr<rclcpp::Node> &node, std::string key, T value)
{
    rclcpp::Parameter p(key, value);
    try {
        node->set_parameter(p);
    } catch (rclcpp::exceptions::ParameterNotDeclaredException &e) {
        node->declare_parameter(key, value);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto parameter = std::make_shared<rclcpp::Node>("parameter");
    std::string cfgpath = ament_index_cpp::get_package_share_directory("params") + "/conf/";
    const std::string root_cfg_file = "config.conf";
    std::string cfgfile(cfgpath+root_cfg_file);
    RCLCPP_INFO(parameter->get_logger(), cfgfile);
    boost::property_tree::ptree pt;
    if(!basic_parser::parse_file(cfgfile, pt)) {
        RCLCPP_ERROR(parameter->get_logger(), "parser cfgfile: %s failed", cfgfile.c_str());
        return false;
    }
    try {
        for(auto p:pt) {
            if(!p.second.data().empty()) {
                std::string data=p.second.data();
                if(p.first.find("file")!=std::string::npos) {
                    data = cfgpath+data;
                }
                if(data == "true") {
                    SetParameter(parameter, p.first, true);
                } else if(data == "false") {
                    SetParameter(parameter, p.first, false);
                } else {
                    try {
                        int d = std::stoi(data);
                        SetParameter(parameter, p.first, d);
                    } catch(std::invalid_argument&) {
                        try {
                            float d = std::stof(data);
                            SetParameter(parameter, p.first, d);
                        } catch(std::invalid_argument&) {
                            SetParameter(parameter, p.first, data);
                        }
                    }
                }
            }
        }
    } catch (boost::property_tree::ptree_error &e) {
        RCLCPP_ERROR(parameter->get_logger(), "%s", e.what());
    }
    rclcpp::spin(parameter);
    return 0;
}
