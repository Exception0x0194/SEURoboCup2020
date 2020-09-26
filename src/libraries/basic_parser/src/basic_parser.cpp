#include <basic_parser/basic_parser.hpp>
#include <fstream>

namespace basic_parser
{

bool get_tree_from_file(const std::string &filename, boost::property_tree::ptree &pt)
{
    std::ifstream ifs(filename.c_str(), std::ios::in);
    std::string line;
    int count_of_quotatuion = 0;
    std::stringstream json_data;

    if (!ifs) {
        return false;
    }

    while (std::getline(ifs, line)) {
        count_of_quotatuion = 0;

        for (uint32_t i = 0; i < line.size(); i++) {
            if (line[i] == '\'' || line[i] == '\"') {
                count_of_quotatuion++;
            }

            if (i < line.size() - 2) {
                if (line[i] == '/' && line[i + 1] == '/' && count_of_quotatuion % 2 == 0) {
                    break;
                }
            }

            json_data << line[i];
        }

        line.clear();
    }

    boost::property_tree::read_json(json_data, pt);
    return true;
}

void write_tree_to_file(const std::string &filename, const boost::property_tree::ptree &pt)
{
    std::ostringstream os;
    boost::property_tree::write_json(os, pt);
    std::ofstream tree(filename.c_str());
    tree << os.str();
    tree.close();
}

bool add_child(boost::property_tree::ptree &oript, const std::string &key, const boost::property_tree::ptree &pt)
{
    try {
        oript.erase(key);
        oript.add_child(key, pt);
    } catch (boost::property_tree::ptree_error &e) {
        return false;
    }

    return true;
}

bool parse_ptree(boost::property_tree::ptree &pt)
{
    std::string data;
    boost::property_tree::ptree tpt;
    boost::property_tree::ptree oript = pt;
    int size = oript.size();

    if (size == 1) {
        data.clear();
        data = oript.begin()->second.data();

        if (data.size() == 0) {
            tpt = oript.begin()->second;

            if (parse_ptree(tpt)) {
                if (!add_child(pt, oript.begin()->first, tpt)) {
                    return false;
                } else {
                    return true;
                }
            } else {
                return false;
            }
        } else {
            if (oript.begin()->first.empty()) {
                if (parse_file(data, tpt)) {
                    pt = tpt;
                } else {
                    return false;
                }

                return true;
            }
        }
    }

    auto iter = oript.begin();

    while (iter != oript.end()) {
        tpt = iter->second;
        data.clear();
        data = tpt.data();

        if (data.size() == 0) {
            if (parse_ptree(tpt)) {
                if (!add_child(pt, iter->first, tpt)) {
                    return false;
                }
            } else {
                return false;
            }
        }

        iter++;
    }

    return true;
}

bool parse_file(const std::string &cfgname, boost::property_tree::ptree &pt)
{
    if (get_tree_from_file(cfgname, pt)) {
        return parse_ptree(pt);
    } else {
        return false;
    }
}
}