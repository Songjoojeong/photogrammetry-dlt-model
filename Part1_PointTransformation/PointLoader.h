#pragma once
#include <vector>
#include <string>

struct Point2D {
    double x;
    double y;
};

struct PointPair {
    Point2D img1;
    Point2D img2;
};

class PointLoader {
public:
    static std::vector<PointPair> LoadFromFile(const std::string& filepath);
    static void SplitPoints(const std::vector<PointPair>& allPoints,
        std::vector<PointPair>& modelPoints,
        std::vector<PointPair>& testPoints,
        double modelRatio = 0.5);
};
