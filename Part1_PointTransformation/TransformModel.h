#pragma once
#include "PointLoader.h"
#include <vector>

// Affine 변환 파라미터 구조체
struct AffineParams {
    double a, b, c;  // x' = ax + by + c
    double d, e, f;  // y' = dx + ey + f
};

// Homography 행렬 구조체
struct HomographyMatrix {
    double h[3][3];
};

// Affine 모델 클래스
class AffineModel {
public:
    static AffineParams Estimate(const std::vector<PointPair>& modelPoints);
    static Point2D Transform(const Point2D& pt, const AffineParams& params);
};

// Homography 모델 클래스
class HomographyModel {
public:
    static HomographyMatrix Estimate(const std::vector<PointPair>& modelPoints);
    static Point2D Transform(const Point2D& pt, const HomographyMatrix& H);
};

// RMSE 계산기 클래스
class ErrorCalculator {
public:
    static double ComputeRMSEAffine(const std::vector<PointPair>& points, const AffineParams& params);
    static double ComputeRMSEHomography(const std::vector<PointPair>& points, const HomographyMatrix& H);
};
