
#ifndef IOU_H
#define IOU_H

#include <vector>
#include <stack>

struct rect_basic_struct
{
    double center_pos[3]; // x,y,z (横/纵/高)
    double box_len;
    double box_wid;
    double box_height;
    double heading;
};

struct rect_box_struct
{
    double x1;
    double y1;

    double x2;
    double y2;

    double x3;
    double y3;

    double x4;
    double y4;
};

struct Point
{
    double x, y;
};

struct rect_corners_struct
{
    Point corners[4];
};


void creat_rect_box_point(const rect_basic_struct & rect_1, rect_corners_struct &box_corners);
double intersection_area(const rect_corners_struct & box_1, const rect_corners_struct & box_2);
Point intersection(Point a,Point b,Point c,Point d);
double calcul_rect_area(const rect_basic_struct &r);
double PolygonArea(Point p[], int n);
double cross(Point a,Point b,Point c);
int dcmp(double x);
double SPIA(Point a[], Point b[], int na, int nb);  
double CPIA(Point a[], Point b[], int na, int nb);  

void find_p0(Point &p0, std::vector<Point> &points);
bool cmp_(Point &p1, Point &p2);
void find_convex_hull(std::vector<Point> points, Point p0);
double compute_convexHullArea(const rect_corners_struct &r1, const rect_corners_struct &r2);

// 计算各类IOU
double IOU_2D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);
double IOU_3D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);
double GIOU_2D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);
double GIOU_3D(const rect_basic_struct &rect_1, const rect_basic_struct &rect_2);


#endif
