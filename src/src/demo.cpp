#include <iostream>
#include <string>
#include <dirent.h>

int main() {
   std::string directory_path = "/home/zdhjs-05/myGitHubCode/radar_scenes/RadarScenes/data/sequence_2/radarpc";// 遍历当前目录
    // 指定要遍历的目录路径
    // std::string directory_path = "/path/to/your/directory";  // 替换为你的目录路径

    // 打开目录
    DIR* dir = opendir(directory_path.c_str());
    if (dir == nullptr) {
        std::cerr << "无法打开目录" << std::endl;
        return 1;
    }

    // 遍历目录下的文件
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG) {  // 检查是否是普通文件
            // 打印文件名
            std::cout << entry->d_name << std::endl;
        }
    }

    // 关闭目录
    closedir(dir);
   return 0;
}


// int main()
// {
//     std::string file_path = "/home/charles/myCode/all_my_code/src/src/test.txt";
//     std::ifstream file2(file_path);

//     if (file2.is_open())
//     {
//         std::string lineStr;
//         std::vector<std::string> lineArray;
//         std::string str_line;

//         std::vector<radar_point_t> radar_meas;
//         while (std::getline(file2, lineStr))
//         {
//             // std::cout << lineStr << std::endl;
//             std::stringstream ss(lineStr);
//             lineArray.clear();

//             radar_point_t radar_pc;
//             while (std::getline(ss, str_line, ' '))
//             {
//                 lineArray.push_back(str_line);
//             }

//             uint idx=0;
//             radar_pc.range_sc = std::atof(lineArray.at(idx).c_str());idx++;
//             radar_pc.azimuth_sc = std::atof(lineArray.at(idx).c_str());idx++;
//             radar_pc.vr = std::atof(lineArray.at(idx).c_str());idx++;
//             radar_pc.rcs = std::atof(lineArray.at(idx).c_str());idx++;
//             radar_pc.vr_compensated = std::atof(lineArray.at(idx).c_str());idx++;
//             radar_pc.x_cc = std::atof(lineArray.at(idx).c_str());idx++;
//             radar_pc.y_cc = std::atof(lineArray.at(idx).c_str());idx++;
//             radar_pc.x_seq = std::atof(lineArray.at(idx).c_str());idx++;
//             radar_pc.y_seq = std::atof(lineArray.at(idx).c_str());idx++;
                            
//             radar_meas.push_back(radar_pc);
//         }

//         std::cout << radar_meas.size() << std::endl;
//     }

//     return 0;
// }
