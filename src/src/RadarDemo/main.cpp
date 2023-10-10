#include <iostream>
#include <vector>
#include <fstream>
#include <filesystem>
#include <algorithm>

#include "type.h"

// Eigen
#include <Eigen/Dense>

#include "radarTracker.h"
using namespace RadarDemo;

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
bool sort_file_by_name(std::string &a, std::string &b)
{
    double aa = std::atof(a.c_str());
    double bb = std::atof(b.c_str());

    return (aa < bb);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {string&} file_path
 * @return {*}
 */
std::vector<radar_point_t> read_radar_point_from_file(const std::string &file_path)
{
    std::vector<radar_point_t> radar_meas;

    std::ifstream file2(file_path);
    if (file2.is_open())
    {
        std::string lineStr;
        std::vector<std::string> lineArray;
        std::string str_line;

        while (std::getline(file2, lineStr))
        {
            // std::cout << lineStr << std::endl;
            std::stringstream ss(lineStr);
            lineArray.clear();

            radar_point_t radar_pc;
            while (std::getline(ss, str_line, ' '))
            {
                lineArray.push_back(str_line);
            }

            uint idx = 0;
            radar_pc.x_cc = std::atof(lineArray.at(idx).c_str());
            idx++;
            radar_pc.y_cc = std::atof(lineArray.at(idx).c_str());
            idx++;
            radar_pc.range_sc = std::atof(lineArray.at(idx).c_str());
            idx++;
            radar_pc.azimuth_sc = std::atof(lineArray.at(idx).c_str());
            idx++;
            radar_pc.rcs = std::atof(lineArray.at(idx).c_str());
            idx++;
            radar_pc.vr = std::atof(lineArray.at(idx).c_str());
            idx++;
            radar_pc.vr_compensated = std::atof(lineArray.at(idx).c_str());
            idx++;
            radar_pc.x_seq = std::atof(lineArray.at(idx).c_str());
            idx++;
            radar_pc.y_seq = std::atof(lineArray.at(idx).c_str());
            idx++;

            radar_meas.push_back(radar_pc);
        }

        // std::cout << radar_meas.size() << std::endl;
    }

    return radar_meas;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
using std::filesystem::directory_iterator; // SUPPORTED FROM C++17
using std::filesystem::path;
int main(int argc, char **argv)
{
    std::cout << "It a SimpleTrack Demo" << std::endl;

    // 遍历点云文件
    // std::string radar_path_pc = "/home/zdhjs-05/myGitHubCode/radar_scenes/RadarScenes/data/sequence_2/radarpc/";
    // std::string image_path_det = "/home/zdhjs-05/myGitHubCode/radar_scenes/RadarScenes/data/sequence_2/camera/";

    std::string radar_path_pc = "/home/charles/myCode/all_my_code/src/src/RadarDemo/radarscene/radarpc/";
    // std::string image_path_det = "/home/zdhjs-05/myGitHubCode/radar_scenes/RadarScenes/data/sequence_2/camera/";

    std::string file_path;
    std::vector<std::string> file_list;
    for (auto &v : directory_iterator(radar_path_pc))
    {
        std::string sub_file = v.path().stem().string();
        file_list.push_back(sub_file);
    }

    std::sort(file_list.begin(), file_list.end(), sort_file_by_name);

    for (auto &file_ : file_list)
    {
        file_path = radar_path_pc + file_ + ".txt";

        std::vector<radar_point_t> radar_meas = read_radar_point_from_file(file_path);

        radar_track_main(radar_meas);
    }

    return 1;
}