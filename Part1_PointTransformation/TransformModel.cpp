#include "TransformModel.h"
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

// ==================== Affine 변환 모델 구현부 ====================

// Affine 파라미터 추정: (x, y) → (x', y')로의 선형 변환 매개변수 6개 추정
AffineParams AffineModel::Estimate(const std::vector<PointPair>& modelPoints) {
    size_t N = modelPoints.size();
    if (N < 3) {
        std::cerr << "[Error] 최소 3개의 대응점이 필요합니다." << std::endl;
        return {};
    }

    Eigen::MatrixXd A(2 * N, 6);   // 변환식의 계수 행렬
    Eigen::VectorXd L(2 * N);      // 목표값 벡터

    for (size_t i = 0; i < N; ++i) {
        const auto& pt1 = modelPoints[i].img1;
        const auto& pt2 = modelPoints[i].img2;

        // x' = ax + by + c
        A(2 * i, 0) = pt1.x;
        A(2 * i, 1) = pt1.y;
        A(2 * i, 2) = 1;
        A(2 * i, 3) = 0;
        A(2 * i, 4) = 0;
        A(2 * i, 5) = 0;

        // y' = dx + ey + f
        A(2 * i + 1, 0) = 0;
        A(2 * i + 1, 1) = 0;
        A(2 * i + 1, 2) = 0;
        A(2 * i + 1, 3) = pt1.x;
        A(2 * i + 1, 4) = pt1.y;
        A(2 * i + 1, 5) = 1;

        L(2 * i)     = pt2.x;
        L(2 * i + 1) = pt2.y;
    }

    // 최소자승 해 계산 (AᵀA)⁻¹AᵀL
    Eigen::VectorXd X = (A.transpose() * A).ldlt().solve(A.transpose() * L);

    AffineParams params;
    params.a = X(0); params.b = X(1); params.c = X(2);
    params.d = X(3); params.e = X(4); params.f = X(5);

    return params;
}

// Affine 변환 적용 함수
Point2D AffineModel::Transform(const Point2D& pt, const AffineParams& params) {
    double xNew = params.a * pt.x + params.b * pt.y + params.c;
    double yNew = params.d * pt.x + params.e * pt.y + params.f;
    return { xNew, yNew };
}

// ==================== Homography 변환 모델 구현부 ====================

// Homography 파라미터 추정: 3x3 행렬로 표현되는 projective 변환
HomographyMatrix HomographyModel::Estimate(const std::vector<PointPair>& modelPoints) {
    size_t N = modelPoints.size();
    if (N < 4) {
        std::cerr << "[Error] 최소 4개의 대응점이 필요합니다." << std::endl;
        return {};
    }

    Eigen::MatrixXd A(2 * N, 8);   // h11~h32까지 8개의 파라미터
    Eigen::VectorXd b(2 * N);

    for (size_t i = 0; i < N; ++i) {
        const auto& p1 = modelPoints[i].img1;
        const auto& p2 = modelPoints[i].img2;

        // x' = (h1*x + h2*y + h3) / (h7*x + h8*y + 1)
        A(2 * i, 0) = p1.x;
        A(2 * i, 1) = p1.y;
        A(2 * i, 2) = 1;
        A(2 * i, 3) = 0;
        A(2 * i, 4) = 0;
        A(2 * i, 5) = 0;
        A(2 * i, 6) = -p1.x * p2.x;
        A(2 * i, 7) = -p1.y * p2.x;
        b(2 * i) = p2.x;

        // y' = (h4*x + h5*y + h6) / (h7*x + h8*y + 1)
        A(2 * i + 1, 0) = 0;
        A(2 * i + 1, 1) = 0;
        A(2 * i + 1, 2) = 0;
        A(2 * i + 1, 3) = p1.x;
        A(2 * i + 1, 4) = p1.y;
        A(2 * i + 1, 5) = 1;
        A(2 * i + 1, 6) = -p1.x * p2.y;
        A(2 * i + 1, 7) = -p1.y * p2.y;
        b(2 * i + 1) = p2.y;
    }

    // 최소자승 해 구하기
    Eigen::VectorXd h = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    HomographyMatrix H;
    H.h[0][0] = h(0); H.h[0][1] = h(1); H.h[0][2] = h(2);
    H.h[1][0] = h(3); H.h[1][1] = h(4); H.h[1][2] = h(5);
    H.h[2][0] = h(6); H.h[2][1] = h(7); H.h[2][2] = 1.0; // 마지막 원소는 1로 고정

    return H;
}

// Homography 변환 적용 함수
Point2D HomographyModel::Transform(const Point2D& pt, const HomographyMatrix& H) {
    double x = pt.x, y = pt.y;
    double denom = H.h[2][0] * x + H.h[2][1] * y + H.h[2][2];

    if (denom == 0.0) {
        std::cerr << "[Warning] 나눗셈 0 오류 발생, 원본 좌표 반환." << std::endl;
        return pt;
    }

    double xNew = (H.h[0][0] * x + H.h[0][1] * y + H.h[0][2]) / denom;
    double yNew = (H.h[1][0] * x + H.h[1][1] * y + H.h[1][2]) / denom;
    return { xNew, yNew };
}

// ==================== RMSE 계산기 구현부 ====================

// Affine 모델 RMSE 계산
double ErrorCalculator::ComputeRMSEAffine(const std::vector<PointPair>& points, const AffineParams& params) {
    double totalError = 0.0;
    for (const auto& pair : points) {
        Point2D pred = AffineModel::Transform(pair.img1, params);
        double dx = pred.x - pair.img2.x;
        double dy = pred.y - pair.img2.y;
        totalError += dx * dx + dy * dy;
    }
    return std::sqrt(totalError / points.size());
}

// Homography 모델 RMSE 계산
double ErrorCalculator::ComputeRMSEHomography(const std::vector<PointPair>& points, const HomographyMatrix& H) {
    double totalError = 0.0;
    for (const auto& pair : points) {
        Point2D pred = HomographyModel::Transform(pair.img1, H);
        double dx = pred.x - pair.img2.x;
        double dy = pred.y - pair.img2.y;
        totalError += dx * dx + dy * dy;
    }
    return std::sqrt(totalError / points.size());
}
