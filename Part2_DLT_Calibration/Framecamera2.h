// Framecamera2.h
#pragma once
#include <string>
#include <vector>

// 3차원 좌표
struct Point3D {
    double X, Y, Z;
};

// 2차원 영상 좌표
struct ImageGCP {
    std::string ImageName;
    double Col; // column
    double Row; // row
};

// GCP
struct GCP {
    Point3D xyz;
    std::vector<ImageGCP> order;    // 해당 GCP의 영상 좌표
};
