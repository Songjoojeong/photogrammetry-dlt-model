#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include "PointLoader.h"
#include "TransformModel.h"

// 변환 모델 평가 함수
void RunEvaluation(const std::string& name, const std::vector<PointPair>& points) 
{ 
    // 1. 대응점을 모델링용/검사용으로 50:50 무작위 분할
    std::vector<PointPair> modelPoints, testPoints; 
    PointLoader::SplitPoints(points, modelPoints, testPoints, 0.5); 

    // 2. Affine 변환 추정 및 파라미터 출력
    AffineParams affine = AffineModel::Estimate(modelPoints); 
    std::cout << "[Affine 파라미터]" << std::endl; 
    std::cout << "a: " << affine.a << ", b: " << affine.b << ", c: " << affine.c << std::endl; 
    std::cout << "d: " << affine.d << ", e: " << affine.e << ", f: " << affine.f << std::endl; 

    // Affine RMSE 계산
    double rmse_model_aff = ErrorCalculator::ComputeRMSEAffine(modelPoints, affine); 
    double rmse_test_aff = ErrorCalculator::ComputeRMSEAffine(testPoints, affine); 

    // 3. Homography 변환 추정 및 행렬 출력
    HomographyMatrix H = HomographyModel::Estimate(modelPoints); 
    std::cout << "[Homography 파라미터]" << std::endl; 
    for (int i = 0; i < 3; ++i) { 
        for (int j = 0; j < 3; ++j) { 
            std::cout << std::setw(12) << H.h[i][j] << " "; 
        } 
        std::cout << std::endl; 
    } 

    // Homography RMSE 계산
    double rmse_model_homo = ErrorCalculator::ComputeRMSEHomography(modelPoints, H); 
    double rmse_test_homo = ErrorCalculator::ComputeRMSEHomography(testPoints, H); 

    // 유효성 검사
    if (affine.a == 0 && affine.b == 0 && affine.c == 0) { 
        std::cerr << "[Error] 유효하지 않은 Affine 파라미터\n"; 
        return; 
    } 

    // 최종 결과 출력
    std::cout << "\n--- [" << name << "] ---" << std::endl; 
    std::cout << std::fixed << std::setprecision(5); 
    std::cout << "[Affine 변환] 모델점 RMSE: " << rmse_model_aff << " | 검사점 RMSE: " << rmse_test_aff << std::endl; 
    std::cout << "[Homography 변환] 모델점 RMSE: " << rmse_model_homo << " | 검사점 RMSE: " << rmse_test_homo << std::endl; 
    std::cout << "---------------------------\n" << std::endl;
} 

int main() { 
    // 대응점 파일 불러오기
    std::vector<PointPair> roofPoints = PointLoader::LoadFromFile("points_roof.txt"); 
    std::vector<PointPair> groundPoints = PointLoader::LoadFromFile("points_ground.txt"); 

    if (roofPoints.empty() || groundPoints.empty()) {
        std::cerr << "데이터 파일을 찾을 수 없습니다. 경로를 확인하세요." << std::endl;
        return -1;
    }

    // A - 지붕 점만 평가
    RunEvaluation("A - 지붕 점만", roofPoints); 

    // B - 바닥 점만 평가
    RunEvaluation("B - 바닥 점만", groundPoints); 

    // C - 지붕 + 바닥 점 전체로 평가
    std::vector<PointPair> combinedPoints = roofPoints; 
    combinedPoints.insert(combinedPoints.end(), groundPoints.begin(), groundPoints.end()); 
    RunEvaluation("C - 지붕 + 바닥", combinedPoints); 

    return 0; 
}
