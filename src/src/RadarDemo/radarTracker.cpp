#include "radarTracker.h"

void point_cluster(std::vector<radar_point_t> &new_meas)
{
    std::vector<std::vector<uint16_t>> clusterSet;
    DBSCAN::Point4DBSCAN Point;
    std::vector<DBSCAN::Point4DBSCAN> PointSet;

    /* 设置点云DBSCAN参数 */
    for (size_t n = 0; n < new_meas.size(); n++)
    {
        auto &sub_meas = new_meas.at(n);

        Point.PointInfo.ID = n;

        Point.PointInfo.DistLat = sub_meas.x_cc;
        Point.PointInfo.DistLong = sub_meas.y_cc;
        
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
    }

    DBSCAN::KNN_DBSCAN(pointSet, clusterSet);
}

int radar_track_main(std::vector<radar_point_t> &new_meas)
{
    // Step 1: 
    point_cluster(new_meas);

    // Step 2: Trace Predict 

    // Step 3: Association

    // Step 4: Trace Update
}