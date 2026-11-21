#include "Framecamera2.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr double ToRad = M_PI / 180.0;

int main() {
    // --- Input image identifier ---
    const std::string targetImage = "R0047627.JPG";

    // =====================================
    // 1. Define GCPs: 6 modeling + 6 checking
    // =====================================
    std::vector<GCP> all = {
        // Modeling GCPs (spatially balanced)
        {{292315.61, 4147477.92, 37.78}, {{targetImage, 4830, 2132}}},
        {{292298.23, 4147376.31, 34.71}, {{targetImage, 1989, 3080}}},
        {{292279.63, 4147393.80, 33.75}, {{targetImage, 2145, 2402}}},
        {{292262.13, 4147467.82, 33.85}, {{targetImage, 3715, 920}}},
        {{292273.54, 4147502.67, 37.53}, {{targetImage, 4849, 652}}},
        {{292224.58, 4147422.45, 33.76}, {{targetImage, 2053, 675}}},
        // Checking GCPs
        {{292282.67, 4147366.16, 34.22}, {{targetImage, 1547, 2834}}},
        {{292212.50, 4147433.76, 33.79}, {{targetImage, 2155, 219}}},
        {{292252.36, 4147441.39, 33.83}, {{targetImage, 2905, 1066}}},
        {{292269.24, 4147432.76, 33.84}, {{targetImage, 2936, 1607}}},
        {{292301.15, 4147432.22, 34.01}, {{targetImage, 3384, 2400}}},
        {{292328.52, 4147431.38, 34.54}, {{targetImage, 3757, 3091}}}
    };

    // Split into modeling and checking sets
    std::vector<GCP> modelGCPs(all.begin(), all.begin() + 6);
    std::vector<GCP> checkGCPs(all.begin() + 6, all.end());

    // ================================
    // 2. Coordinate normalization (offset)
    // ================================
    const double offsetX = 292000;
    const double offsetY = 4147000;
    const double offsetZ = 30;
    for (auto& g : modelGCPs) {
        g.xyz.X -= offsetX;
        g.xyz.Y -= offsetY;
        g.xyz.Z -= offsetZ;
    }
    for (auto& g : checkGCPs) {
        g.xyz.X -= offsetX;
        g.xyz.Y -= offsetY;
        g.xyz.Z -= offsetZ;
    }

    int N = static_cast<int>(modelGCPs.size());

    // =====================================
    // 3. Direct DLT: estimate P (3x4)
    // =====================================
    cv::Mat A(2 * N, 12, CV_64F);
    for (int i = 0; i < N; ++i) {
        double X = modelGCPs[i].xyz.X;
        double Y = modelGCPs[i].xyz.Y;
        double Z = modelGCPs[i].xyz.Z;
        double c = modelGCPs[i].order[0].Col;
        double r = modelGCPs[i].order[0].Row;
        // first row for x
        cv::Mat row1 = (cv::Mat_<double>(1, 12) <<
            X, Y, Z, 1, 0, 0, 0, 0, -c * X, -c * Y, -c * Z, -c);
        // second row for y
        cv::Mat row2 = (cv::Mat_<double>(1, 12) <<
            0, 0, 0, 0, X, Y, Z, 1, -r * X, -r * Y, -r * Z, -r);
        row1.copyTo(A.row(2 * i));
        row2.copyTo(A.row(2 * i + 1));
    }
    // Solve via SVD
    cv::Mat s, u, vt;
    cv::SVD::compute(A, s, u, vt);
    cv::Mat P = vt.row(11).reshape(0, 3); // last row -> projection matrix

    // Output Direct DLT parameters and RMSE
    std::cout << "\n[Direct DLT Projection Matrix P]" << std::endl;
    std::cout << P << std::endl;

    double err_sum = 0;
    for (auto& g : checkGCPs) {
        cv::Mat pt = (cv::Mat_<double>(4, 1) << g.xyz.X, g.xyz.Y, g.xyz.Z, 1);
        cv::Mat pr = P * pt;
        double c_est = pr.at<double>(0) / pr.at<double>(2);
        double r_est = pr.at<double>(1) / pr.at<double>(2);
        double dx = c_est - g.order[0].Col;
        double dy = r_est - g.order[0].Row;
        err_sum += dx * dx + dy * dy;
    }
    double rmse_direct = std::sqrt(err_sum / checkGCPs.size());
    std::cout << "[Direct DLT RMSE] " << rmse_direct << " px\n";

    // =====================================
    // 4. Indirect DLT: estimate coefficients
    // =====================================
    const double pixel_mm = 0.00479;
    const int w = 4928, h = 3264;
    cv::Mat B(2 * N, 11, CV_64F), L(2 * N, 1, CV_64F);
    for (int i = 0; i < N; ++i) {
        double X = modelGCPs[i].xyz.X;
        double Y = modelGCPs[i].xyz.Y;
        double Z = modelGCPs[i].xyz.Z;
        double c = modelGCPs[i].order[0].Col;
        double r = modelGCPs[i].order[0].Row;
        double x = (c - w / 2.0) * pixel_mm;
        double y = -(r - h / 2.0) * pixel_mm;
        // fill B and L
        B.at<double>(2 * i, 0) = X;      B.at<double>(2 * i, 1) = Y;      B.at<double>(2 * i, 2) = Z;
        B.at<double>(2 * i, 3) = 1;      B.at<double>(2 * i, 4) = 0;      B.at<double>(2 * i, 5) = 0;
        B.at<double>(2 * i, 6) = 0;      B.at<double>(2 * i, 7) = 0;      B.at<double>(2 * i, 8) = -x * X;
        B.at<double>(2 * i, 9) = -x * Y; B.at<double>(2 * i, 10) = -x * Z;
        L.at<double>(2 * i, 0) = x;
        B.at<double>(2 * i + 1, 0) = 0;      B.at<double>(2 * i + 1, 1) = 0;      B.at<double>(2 * i + 1, 2) = 0;
        B.at<double>(2 * i + 1, 3) = 0;      B.at<double>(2 * i + 1, 4) = X;      B.at<double>(2 * i + 1, 5) = Y;
        B.at<double>(2 * i + 1, 6) = Z;      B.at<double>(2 * i + 1, 7) = 1;      B.at<double>(2 * i + 1, 8) = -y * X;
        B.at<double>(2 * i + 1, 9) = -y * Y; B.at<double>(2 * i + 1, 10) = -y * Z;
        L.at<double>(2 * i + 1, 0) = y;
    }
    // least squares solve
    cv::Mat coeffs;
    cv::solve(B.t() * B, B.t() * L, coeffs, cv::DECOMP_SVD);

    // reconstruct and output Indirect DLT projection matrix
    cv::Mat Pxy = (cv::Mat_<double>(3, 4) <<
        coeffs.at<double>(0), coeffs.at<double>(1), coeffs.at<double>(2), coeffs.at<double>(3),
        coeffs.at<double>(4), coeffs.at<double>(5), coeffs.at<double>(6), coeffs.at<double>(7),
        coeffs.at<double>(8), coeffs.at<double>(9), coeffs.at<double>(10), 1.0);

    std::cout << "\n[Indirect DLT Coefficients]" << std::endl;
    for (int i = 0; i < coeffs.rows; ++i)
        std::cout << "a" << i << " = " << coeffs.at<double>(i) << std::endl;

    std::cout << "\n[Indirect DLT Projection Matrix Pxy]" << std::endl;
    std::cout << Pxy << std::endl;

    // calculate Indirect RMSE
    err_sum = 0;
    for (auto& g : checkGCPs) {
        cv::Mat pt = (cv::Mat_<double>(4, 1) << g.xyz.X, g.xyz.Y, g.xyz.Z, 1);
        cv::Mat pr = Pxy * pt;
        double x = pr.at<double>(0) / pr.at<double>(2);
        double y = pr.at<double>(1) / pr.at<double>(2);
        double c_est = x / pixel_mm + w / 2.0;
        double r_est = -y / pixel_mm + h / 2.0;
        double dx = c_est - g.order[0].Col;
        double dy = r_est - g.order[0].Row;
        err_sum += dx * dx + dy * dy;
    }
    double rmse_indirect = std::sqrt(err_sum / checkGCPs.size());
    std::cout << "[Indirect DLT RMSE] " << rmse_indirect << " px\n";

    return 0;
}
