#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <eigen3/Eigen/Dense>

namespace basic_parser
{

bool get_tree_from_file(const std::string &filename, boost::property_tree::ptree &pt);
void write_tree_to_file(const std::string &filename, const boost::property_tree::ptree &pt);

bool parse_file(const std::string &cfgname, boost::property_tree::ptree &pt);

template<typename T, int size>
inline Eigen::Matrix<T, size, 1> get_config_vector(boost::property_tree::ptree &pt, const std::string &keyword)
{
    Eigen::Matrix<T, size, 1> res = Eigen::Matrix<T, size, 1>::Zero(size, 1);
    int i=0;
    boost::property_tree::ptree tpt = pt.get_child(keyword);
    for(auto &t:tpt) {
        if(i>=size) break;
        res[i++] = t.second.get_value<T>();
    }
    return res;
}
}
