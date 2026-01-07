#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

struct Point3D { double X, Y, Z; };
struct ImageGCP { std::string name; double Col, Row; };
struct GCP { Point3D xyz; ImageGCP img; };

struct IOP {
    double focal, pixelSize, PPx, PPy, k1, k2;
};

struct EOP {
    double Xs, Ys, Zs, omega, phi, kappa;
};

cv::Mat BuildRotation(double omega, double phi, double kappa);
void InverseMapping(const EOP& eop, const IOP& iop, const std::vector<GCP>& gcps, std::vector<std::pair<double, double>>& proj);
double ComputeMappingError(const std::vector<GCP>& gcps, const std::vector<std::pair<double, double>>& proj);
void RefineSensorModel(EOP& eop, const std::vector<GCP>& modelGCPs, const IOP& iop, int maxIter = 20, double tol = 1e-6);
