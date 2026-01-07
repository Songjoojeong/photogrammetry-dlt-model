#pragma once
#include <string>
#include <vector>

struct Point3D {
    double X, Y, Z;
};

struct ImageGCP {
    std::string ImageName;
    double Col, Row;
};

struct GCP {
    Point3D xyz;
    std::vector<ImageGCP> order;
};
