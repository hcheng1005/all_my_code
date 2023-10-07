#include <iostream>
#include <vector>
#include <algorithm>

struct Point {
    int x, y;
};

int orientation(Point p, Point q, Point r) {
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0; // Collinear
    return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
}

bool comparePoints(Point p1, Point p2) {
    return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);
}

std::vector<Point> convexHull(std::vector<Point> points) {
    int n = points.size();
    if (n < 3) return points;

    std::vector<Point> hull;

    int l = 0;
    for (int i = 1; i < n; ++i) {
        if (points[i].x < points[l].x) {
            l = i;
        }
    }

    int p = l, q;
    do {
        hull.push_back(points[p]);
        q = (p + 1) % n;
        for (int i = 0; i < n; ++i) {
            if (orientation(points[p], points[i], points[q]) == 2) {
                q = i;
            }
        }
        p = q;
    } while (p != l);

    return hull;
}

int main() {
    std::vector<Point> points = {{1, 1}, {2, 2}, {3, 3}, {4, 4}, {1, 4}, {2, 3}, {3, 2}, {4, 1}};

    std::sort(points.begin(), points.end(), comparePoints);
    std::vector<Point> hull = convexHull(points);

    // 计算凸包的面积（假设凸包至少有三个点）
    int area = 0;
    int n = hull.size();
    for (int i = 0; i < n - 1; ++i) {
        area += (hull[i].x * hull[i + 1].y - hull[i + 1].x * hull[i].y);
    }
    area += (hull[n - 1].x * hull[0].y - hull[0].x * hull[n - 1].y);
    area = std::abs(area) / 2;

    std::cout << "凸包的面积为：" << area << std::endl;

    return 0;
}
