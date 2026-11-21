// ----------------------- FrameCamera.cpp -----------------------
#include "FrameCamera.h"
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static const double ToRad = M_PI / 180.0;

// BuildRotation: κ→φ→ω 회전 순서로 카메라 회전행렬 계산
cv::Mat BuildRotation(double omega, double phi, double kappa) {
    double cO = cos(omega), sO = sin(omega);
    double cP = cos(phi), sP = sin(phi);
    double cK = cos(kappa), sK = sin(kappa);
    cv::Mat R_omega = (cv::Mat_<double>(3, 3) <<
        1, 0, 0, 0, cO, sO, 0, -sO, cO);
    cv::Mat R_phi = (cv::Mat_<double>(3, 3) <<
        cP, 0, -sP, 0, 1, 0, sP, 0, cP);
    cv::Mat R_kappa = (cv::Mat_<double>(3, 3) <<
        cK, sK, 0, -sK, cK, 0, 0, 0, 1);
    return R_kappa * R_phi * R_omega;
}

// InverseMapping: 모든 GCP에 대해 투영 좌표 계산
void InverseMapping(
    const EOP& eop,
    const IOP& iop,
    const std::vector<GCP>& gcps,
    std::vector<std::pair<double, double>>& proj
) {
    proj.clear();
    cv::Mat R = BuildRotation(eop.omega, eop.phi, eop.kappa);
    for (const auto& g : gcps) {
        // world → camera coords
        cv::Mat vec = (cv::Mat_<double>(3, 1) <<
            g.xyz.X - eop.Xs,
            g.xyz.Y - eop.Ys,
            g.xyz.Z - eop.Zs);
        cv::Mat pc = R * vec;
        double xn = pc.at<double>(0) / pc.at<double>(2);
        double yn = pc.at<double>(1) / pc.at<double>(2);
        // radial distortion
        double r2 = xn * xn + yn * yn;
        double sc = 1 + iop.k1 * r2 + iop.k2 * r2 * r2;
        double x_dist = xn * sc;
        double y_dist = yn * sc;
        // metric → pixel
        double x_mm = -iop.focal * x_dist;
        double y_mm = -iop.focal * y_dist;
        double col = iop.PPx + x_mm / iop.pixelSize;
        double row = iop.PPy - y_mm / iop.pixelSize;
        proj.emplace_back(col, row);
    }
}

// ComputeMappingError: RMSE 계산
double ComputeMappingError(
    const std::vector<GCP>& gcps,
    const std::vector<std::pair<double, double>>& proj
) {
    double sum2 = 0;
    for (size_t i = 0; i < gcps.size(); ++i) {
        double dc = gcps[i].img.Col - proj[i].first;
        double dr = gcps[i].img.Row - proj[i].second;
        sum2 += dc * dc + dr * dr;
    }
    return std::sqrt(sum2 / gcps.size());
}

// RefineSensorModel: EOP 6개 파라미터 Gauss-Newton으로 보정
void RefineSensorModel(
    EOP& eop,
    const std::vector<GCP>& modelGCPs,
    const IOP& iop,
    int maxIter,
    double tol
) {
    int m = (int)modelGCPs.size();
    int U = 6; // Xs,Ys,Zs,omega,phi,kappa
    double h = 1e-5;
    for (int it = 0; it < maxIter; ++it) {
        cv::Mat A = cv::Mat::zeros(2 * m, U, CV_64F);
        cv::Mat L = cv::Mat::zeros(2 * m, 1, CV_64F);
        // compute residuals
        std::vector<std::pair<double, double>> proj0;
        InverseMapping(eop, iop, modelGCPs, proj0);
        for (int i = 0; i < m; ++i) {
            L.at<double>(2 * i, 0) = modelGCPs[i].img.Col - proj0[i].first;
            L.at<double>(2 * i + 1, 0) = modelGCPs[i].img.Row - proj0[i].second;
        }
        // compute Jacobian
        for (int j = 0; j < U; ++j) {
            EOP eop_p = eop;
            if (j == 0) eop_p.Xs += h;
            else if (j == 1) eop_p.Ys += h;
            else if (j == 2) eop_p.Zs += h;
            else if (j == 3) eop_p.omega += h;
            else if (j == 4) eop_p.phi += h;
            else eop_p.kappa += h;
            std::vector<std::pair<double, double>> proj_p;
            InverseMapping(eop_p, iop, modelGCPs, proj_p);
            for (int i = 0; i < m; ++i) {
                A.at<double>(2 * i, j) = (proj_p[i].first - proj0[i].first) / h;
                A.at<double>(2 * i + 1, j) = (proj_p[i].second - proj0[i].second) / h;
            }
        }
        // solve normal equations
        cv::Mat dX;
        cv::solve(A.t() * A, A.t() * L, dX, cv::DECOMP_SVD);
        // update parameters
        eop.Xs += dX.at<double>(0);
        eop.Ys += dX.at<double>(1);
        eop.Zs += dX.at<double>(2);
        eop.omega += dX.at<double>(3);
        eop.phi += dX.at<double>(4);
        eop.kappa += dX.at<double>(5);
        if (cv::norm(dX) < tol) break;
    }
}

// -------------------------- main ----------------------------
int main() {
    std::vector<GCP> all;
    auto add = [&](double X, double Y, double Z, double c, double r)
        { all.push_back({ {X,Y,Z},{"img",c,r} }); };
    // 모델링용 GCP 5개 (spatially balanced)
    add(292315.61, 4147477.92, 37.78, 4830, 2132);
    add(292298.23, 4147376.31, 34.71, 1989, 3080);
    add(292279.63, 4147393.80, 33.75, 2145, 2402);
    add(292262.13, 4147467.82, 33.85, 3715, 920);
    add(292273.54, 4147502.67, 37.53, 4849, 652);
    // 검사용 GCP 나머지 7개
    add(292224.58, 4147422.45, 33.76, 2053, 675);
    add(292282.67, 4147366.16, 34.22, 1547, 2834);
    add(292212.50, 4147433.76, 33.79, 2155, 219);
    add(292252.36, 4147441.39, 33.83, 2905, 1066);
    add(292269.24, 4147432.76, 33.84, 2936, 1607);
    add(292301.15, 4147432.22, 34.01, 3384, 2400);
    add(292328.52, 4147431.38, 34.54, 3757, 3091);

    std::vector<GCP> model(all.begin(), all.begin() + 5);
    std::vector<GCP> check(all.begin() + 5, all.end());

    IOP iop{ 18.0, 0.00479, 4928 / 2.0, 3264 / 2, 5.09965342e-009, -4.19479528e-016 };
    EOP eop{ 292263.634317, 4147429.990425, 167.204506,
             -5.228112 * ToRad, 1.377259 * ToRad, 59.684709 * ToRad };

    std::vector<std::pair<double, double>> proj;
    InverseMapping(eop, iop, model, proj);
    std::cout << "Initial Model Error: " << ComputeMappingError(model, proj) << " px\n";
    InverseMapping(eop, iop, check, proj);
    std::cout << "Initial Check Error: " << ComputeMappingError(check, proj) << " px\n\n";

    std::cout << "Initial EOP Parameters:\n"
        << " Xs=" << eop.Xs << ", Ys=" << eop.Ys << ", Zs=" << eop.Zs << "\n"
        << " omega=" << eop.omega << ", phi=" << eop.phi << ", kappa=" << eop.kappa << "\n\n";

    RefineSensorModel(eop, model, iop);

    InverseMapping(eop, iop, model, proj);
    std::cout << "Refined Model Error: " << ComputeMappingError(model, proj) << " px\n";
    InverseMapping(eop, iop, check, proj);
    std::cout << "Refined Check Error: " << ComputeMappingError(check, proj) << " px\n\n";

    std::cout << "Refined EOP Parameters:\n"
        << " Xs=" << eop.Xs << ", Ys=" << eop.Ys << ", Zs=" << eop.Zs << "\n"
        << " omega=" << eop.omega << ", phi=" << eop.phi << ", kappa=" << eop.kappa << "\n";

    return 0;
}
