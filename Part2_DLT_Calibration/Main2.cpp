#include "Framecamera2.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    const std::string targetImage = "R0047627.JPG";
    std::vector<GCP> all = {
        {{292315.61, 4147477.92, 37.78}, {{targetImage, 4830, 2132}}},
        {{292298.23, 4147376.31, 34.71}, {{targetImage, 1989, 3080}}},
        {{292279.63, 4147393.80, 33.75}, {{targetImage, 2145, 2402}}},
        {{292262.13, 4147467.82, 33.85}, {{targetImage, 3715, 920}}},
        {{292273.54, 4147502.67, 37.53}, {{targetImage, 4849, 652}}},
        {{292224.58, 4147422.45, 33.76}, {{targetImage, 2053, 675}}},
        {{292282.67, 4147366.16, 34.22}, {{targetImage, 1547, 2834}}},
        {{292212.50, 4147433.76, 33.79}, {{targetImage, 2155, 219}}},
        {{292252.36, 4147441.39, 33.83}, {{targetImage, 2905, 1066}}},
        {{292269.24, 4147432.76, 33.84}, {{targetImage, 2936, 1607}}},
        {{292301.15, 4147432.22, 34.01}, {{targetImage, 3384, 2400}}},
        {{292328.52, 4147431.38, 34.54}, {{targetImage, 3757, 3091}}}
    };

    std::vector<GCP> modelGCPs(all.begin(), all.begin() + 6);
    std::vector<GCP> checkGCPs(all.begin() + 6, all.end());

    const double offsetX = 292000, offsetY = 4147000, offsetZ = 30;
    for (auto& g : modelGCPs) { g.xyz.X -= offsetX; g.xyz.Y -= offsetY; g.xyz.Z -= offsetZ; }
    for (auto& g : checkGCPs) { g.xyz.X -= offsetX; g.xyz.Y -= offsetY; g.xyz.Z -= offsetZ; }

    int N = (int)modelGCPs.size();
    cv::Mat A(2 * N, 12, CV_64F);
    for (int i = 0; i < N; ++i) {
        double X = modelGCPs[i].xyz.X, Y = modelGCPs[i].xyz.Y, Z = modelGCPs[i].xyz.Z;
        double c = modelGCPs[i].order[0].Col, r = modelGCPs[i].order[0].Row;
        cv::Mat row1 = (cv::Mat_<double>(1, 12) << X, Y, Z, 1, 0, 0, 0, 0, -c*X, -c*Y, -c*Z, -c);
        cv::Mat row2 = (cv::Mat_<double>(1, 12) << 0, 0, 0, 0, X, Y, Z, 1, -r*X, -r*Y, -r*Z, -r);
        row1.copyTo(A.row(2 * i)); row2.copyTo(A.row(2 * i + 1));
    }

    cv::Mat w, u, vt;
    cv::SVD::compute(A, w, u, vt);
    cv::Mat P = vt.row(11).reshape(0, 3);
    std::cout << "\n[Direct DLT Matrix P]\n" << P << std::endl;

    return 0;
}
