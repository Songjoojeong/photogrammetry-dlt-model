#include "TransformModel.h"
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

AffineParams AffineModel::Estimate(const std::vector<PointPair>& modelPoints) {
    size_t N = modelPoints.size();
    if (N < 3) return {};

    Eigen::MatrixXd A(2 * N, 6);
    Eigen::VectorXd L(2 * N);

    for (size_t i = 0; i < N; ++i) {
        const auto& pt1 = modelPoints[i].img1;
        const auto& pt2 = modelPoints[i].img2;
        A(2 * i, 0) = pt1.x; A(2 * i, 1) = pt1.y; A(2 * i, 2) = 1;
        A(2 * i, 3) = 0;     A(2 * i, 4) = 0;     A(2 * i, 5) = 0;
        A(2 * i + 1, 0) = 0; A(2 * i + 1, 1) = 0; A(2 * i + 1, 2) = 0;
        A(2 * i + 1, 3) = pt1.x; A(2 * i + 1, 4) = pt1.y; A(2 * i + 1, 5) = 1;
        L(2 * i) = pt2.x; L(2 * i + 1) = pt2.y;
    }

    Eigen::VectorXd X = A.colPivHouseholderQr().solve(L);
    return {X(0), X(1), X(2), X(3), X(4), X(5)};
}

Point2D AffineModel::Transform(const Point2D& pt, const AffineParams& params) {
    return {params.a * pt.x + params.b * pt.y + params.c, params.d * pt.x + params.e * pt.y + params.f};
}

HomographyMatrix HomographyModel::Estimate(const std::vector<PointPair>& modelPoints) {
    size_t N = modelPoints.size();
    if (N < 4) return {};

    Eigen::MatrixXd A(2 * N, 8);
    Eigen::VectorXd b(2 * N);

    for (size_t i = 0; i < N; ++i) {
        const auto& p1 = modelPoints[i].img1;
        const auto& p2 = modelPoints[i].img2;
        A.row(2 * i) << p1.x, p1.y, 1, 0, 0, 0, -p1.x * p2.x, -p1.y * p2.x;
        A.row(2 * i + 1) << 0, 0, 0, p1.x, p1.y, 1, -p1.x * p2.y, -p1.y * p2.y;
        b(2 * i) = p2.x; b(2 * i + 1) = p2.y;
    }

    Eigen::VectorXd h = A.colPivHouseholderQr().solve(b);
    HomographyMatrix H;
    H.h[0][0] = h(0); H.h[0][1] = h(1); H.h[0][2] = h(2);
    H.h[1][0] = h(3); H.h[1][1] = h(4); H.h[1][2] = h(5);
    H.h[2][0] = h(6); H.h[2][1] = h(7); H.h[2][2] = 1.0;
    return H;
}

Point2D HomographyModel::Transform(const Point2D& pt, const HomographyMatrix& H) {
    double denom = H.h[2][0] * pt.x + H.h[2][1] * pt.y + H.h[2][2];
    if (std::abs(denom) < 1e-10) return pt;
    return {(H.h[0][0] * pt.x + H.h[0][1] * pt.y + H.h[0][2]) / denom, (H.h[1][0] * pt.x + H.h[1][1] * pt.y + H.h[1][2]) / denom};
}

double ErrorCalculator::ComputeRMSEAffine(const std::vector<PointPair>& points, const AffineParams& params) {
    double err = 0;
    for (const auto& p : points) {
        Point2D res = AffineModel::Transform(p.img1, params);
        err += std::pow(res.x - p.img2.x, 2) + std::pow(res.y - p.img2.y, 2);
    }
    return std::sqrt(err / points.size());
}

double ErrorCalculator::ComputeRMSEHomography(const std::vector<PointPair>& points, const HomographyMatrix& H) {
    double err = 0;
    for (const auto& p : points) {
        Point2D res = HomographyModel::Transform(p.img1, H);
        err += std::pow(res.x - p.img2.x, 2) + std::pow(res.y - p.img2.y, 2);
    }
    return std::sqrt(err / points.size());
}
