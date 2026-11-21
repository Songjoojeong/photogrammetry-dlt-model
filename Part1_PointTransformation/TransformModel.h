#pragma once
#include "PointLoader.h"
#include <vector>

// Affine 변환 모델 정의
// (x, y) → (x', y') 로 변환하는 6개 파라미터
struct AffineParams {
    double a, b, c; // x' = ax + by + c
    double d, e, f; // y' = dx + ey + f
};

// Homography 변환 모델 정의
// 3x3 행렬을 이용한 projective 변환
struct HomographyMatrix {
    double h[3][3]; // x', y' = H * [x, y, 1]^T
};

// Affine 변환
// 변환된 좌표 반환
class AffineModel {
public:
    static AffineParams Estimate(const std::vector<PointPair>& modelPoints);
    static Point2D Transform(const Point2D& pt, const AffineParams& params);
};

// Homography 변환
// 좌표 정규화 포함하여 변환 수행
class HomographyModel {
public:
    static HomographyMatrix Estimate(const std::vector<PointPair>& modelPoints);
    static Point2D Transform(const Point2D& pt, const HomographyMatrix& H);
};

// RMSE 계산
class ErrorCalculator {
public:
    static double ComputeRMSEAffine(const std::vector<PointPair>& points, const AffineParams& params);
    static double ComputeRMSEHomography(const std::vector<PointPair>& points, const HomographyMatrix& H);
};
