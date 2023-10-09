#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>

typedef struct
{
    float range_sc;
    float azimuth_sc;
    float vr;
    float rcs;
    float vr_compensated;
    float x_cc;
    float y_cc;
    float x_seq;
    float y_seq;
} radar_point_t;

int main()
{
    std::string file_path = "/home/charles/myCode/all_my_code/src/src/test.txt";
    std::ifstream file2(file_path);

    if (file2.is_open())
    {
        std::string lineStr;
        std::vector<std::string> lineArray;
        std::string str_line;

        std::vector<radar_point_t> radar_meas;
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

            uint idx=0;
            radar_pc.range_sc = std::atof(lineArray.at(idx).c_str());idx++;
            radar_pc.azimuth_sc = std::atof(lineArray.at(idx).c_str());idx++;
            radar_pc.vr = std::atof(lineArray.at(idx).c_str());idx++;
            radar_pc.rcs = std::atof(lineArray.at(idx).c_str());idx++;
            radar_pc.vr_compensated = std::atof(lineArray.at(idx).c_str());idx++;
            radar_pc.x_cc = std::atof(lineArray.at(idx).c_str());idx++;
            radar_pc.y_cc = std::atof(lineArray.at(idx).c_str());idx++;
            radar_pc.x_seq = std::atof(lineArray.at(idx).c_str());idx++;
            radar_pc.y_seq = std::atof(lineArray.at(idx).c_str());idx++;
                            
            radar_meas.push_back(radar_pc);
        }

        std::cout << radar_meas.size() << std::endl;
    }

    return 0;
}
