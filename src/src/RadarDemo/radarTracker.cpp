#include "radarTracker.h"
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using namespace RadarDemo;

void point_cluster(std::vector<radar_point_t> &new_meas)
{
    std::vector<std::vector<size_t>> clusterSet;
    DBSCAN::Point4DBSCAN Point;
    std::vector<DBSCAN::Point4DBSCAN> PointSet;
    cv::Mat image = cv::Mat::zeros(600, 800, CV_8UC3); // 宽800，高600，3通道图像
    /* 设置点云DBSCAN参数 */
    std::vector<cv::Point2f> point_cloud_orin;
    for (size_t n = 0; n < new_meas.size(); n++)
    {
        auto &sub_meas = new_meas.at(n);

        if ((sub_meas.x_cc < 0.0) || (fabs(sub_meas.vr_compensated) < 0.1))
        {
            continue;
        }

        Point.PointInfo.ID = n;

        Point.PointInfo.DistLat = sub_meas.y_cc;
        Point.PointInfo.DistLong = sub_meas.x_cc;

        Point.PointInfo.Range = sub_meas.range_sc;
        Point.PointInfo.Azi = sub_meas.azimuth_sc;
        Point.PointInfo.V = sub_meas.vr;
        Point.PointInfo.RCS = sub_meas.rcs;

        // Point.PointInfo.DynProp = fifo_point[n].DynProp;
        Point.PointInfo.valid = true;
        // Point.PointInfo.prob_exit = fifo_point[n].ProbOfExist;

        Point.DBSCAN_para.Search_R = 2.5F;
        Point.DBSCAN_para.minPts = 2;
        Point.DBSCAN_para.pointType = 255;
        Point.DBSCAN_para.static_or_dyna = 0;

        PointSet.push_back(Point);

        // 添加点到点云数据
        cv::circle(image,
                   cv::Point2f((Point.PointInfo.DistLat + 100) / 200 * 800, 600 - Point.PointInfo.DistLong / 100 * 600),
                   3, cv::Scalar(200, 200, 200), -1); // 红色点
    }
    // std::cout << "DO KNN_DBSCAN " << std::endl;

    // Step 1: cluster
    DBSCAN::KNN_DBSCAN(PointSet, clusterSet);

    // std::cout << "Cluster Result:[ " << clusterSet.size() << " ]" << std::endl;///////
    // cv::Mat image = cv::Mat::zeros(600, 800, CV_8UC3); // 宽800，高600，3通道图像
    int idx = 0;
    for (auto &sub_cluster : clusterSet)
    {
        // std::cout << "Cluster Member size:[ " << sub_cluster.size() << " ]" << std::endl;
        std::vector<cv::Point2f> point_cloud;
        for (auto &pc : sub_cluster)
        {
            // std::cout << "pc info: ["
            //           << PointSet.at(pc).PointInfo.DistLat << ", "
            //           << PointSet.at(pc).PointInfo.DistLong << " ]" << std::endl;

            // 添加点到点云数据
            point_cloud.push_back(cv::Point2f((PointSet.at(pc).PointInfo.DistLat + 100) / 200 * 800,
                                              600 - PointSet.at(pc).PointInfo.DistLong / 100 * 600));
        }

        // 显示图像
        for (const cv::Point2f &point : point_cloud)
        {
            cv::circle(image, point, 2, cv::Scalar(0, idx * 20, 50 + idx * 20), -1); // 红色点
        }

        idx++;
    }

    // Step 2: box fitting
    std::vector<radar_cluster_t> radar_cluters;
    for (auto &sub_cluster : clusterSet)
    {
        radar_cluster_t radar_cluster;
        radar_cluster.pc_idx = sub_cluster;

        std::vector<T> len_vec, wid_vec;
        for (const auto &pc : radar_cluster.pc_idx)
        {
            len_vec.push_back(PointSet.at(pc).PointInfo.DistLong);
            wid_vec.push_back(PointSet.at(pc).PointInfo.DistLat);
        }

        auto l = std::minmax_element(len_vec.begin(), len_vec.end());
        auto w = std::minmax_element(wid_vec.begin(), wid_vec.end());

        radar_cluster.center[1] = (*l.first + *l.second) * 0.5;
        radar_cluster.center[0] = (*w.first + *w.second) * 0.5;

        radar_cluster.len = *l.second - *l.first;
        radar_cluster.wid = *w.second - *w.first;

        if (radar_cluster.len < 0.5)
        {
            radar_cluster.len = 0.5;
        }
        if (radar_cluster.wid < 0.5)
        {
            radar_cluster.wid = 0.5;
        }

        // std::cout << "Center Pos: " << radar_cluster.center[0] << ", "
        //           << radar_cluster.center[1] << "] " << std::endl;

        radar_cluters.push_back(radar_cluster);

        cv::Rect rect((radar_cluster.center[0] - radar_cluster.wid * 0.5 + 100) / 200 * 800,
                      600 - (radar_cluster.center[1] + radar_cluster.len * 0.5) / 100 * 600,
                      radar_cluster.wid / 200 * 800,
                      radar_cluster.len / 100 * 600); // 定义矩形框，左上角坐标 (200, 200)，宽高 (200, 200)

        cv::rectangle(image, rect, cv::Scalar(125, 125, 125), 2); // 绿色矩形框
    }

    cv::imshow("Point Cloud Visualization", image);
    cv::waitKey(100);
}

int radar_track_main(std::vector<radar_point_t> &new_meas)
{
    // Step 1:
    point_cluster(new_meas);

    // Step 2: Trace Predict

    // Step 3: Association

    // Step 4: Trace Update

    return 1;
}