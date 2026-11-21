// ----------------------- FrameCamera.h -----------------------
#ifndef FRAMECAMERA_H
#define FRAMECAMERA_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

// 지상좌표점
struct Point3D { double X, Y, Z; };
// GCP 측정값
struct ImageGCP { std::string name; double Col, Row; };
// GCP 결합
struct GCP { Point3D xyz; ImageGCP img; };

// 내부표정요소 (IOP) 정의
struct IOP {
    double focal;      // 초점거리 [mm]
    double pixelSize;  // 픽셀 크기 [mm/pixel]
    double PPx, PPy;   // 주점 좌표 [pixels]
    double k1, k2;     // 방사 왜곡 계수
};

// 외부표정요소 (EOP) 정의
struct EOP {
    double Xs, Ys, Zs;         // 카메라 위치 [world]
    double omega, phi, kappa;  // 회전 각도 [rad]
};

// ω, φ, κ 순서의 회전 행렬 생성 함수
cv::Mat BuildRotation(double omega, double phi, double kappa);

// InverseMapping: 지상점 → 영상좌표 변환 (방사왜곡 포함)
void InverseMapping(
    const EOP& eop,
    const IOP& iop,
    const std::vector<GCP>& gcps,
    std::vector<std::pair<double, double>>& proj
);

// ComputeMappingError: 관측 vs 모델링 RMSE 계산
double ComputeMappingError(
    const std::vector<GCP>& gcps,
    const std::vector<std::pair<double, double>>& proj
);

// RefineSensorModel: 모델 GCP로 Gauss-Newton 보정 수행
void RefineSensorModel(
    EOP& eop,
    const std::vector<GCP>& modelGCPs,
    const IOP& iop,
    int maxIter = 20,
    double tol = 1e-6
);

#endif // FRAMECAMERA_H
