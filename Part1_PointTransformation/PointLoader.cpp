#include "PointLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>
#include <algorithm>

std::vector<PointPair> PointLoader::LoadFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    std::vector<PointPair> points;
    if (!file.is_open()) {
        std::cerr << "[Error] 파일 열기 실패: " << filepath << std::endl;
        return points;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double x1, y1, x2, y2;
        char comma;

        if (iss >> x1 >> comma >> y1 >> comma >> x2 >> comma >> y2) {
            Point2D p1 = { x1, y1 };
            Point2D p2 = { x2, y2 };
            points.push_back({ p1, p2 });
        }
    }

    std::cout << "[Info] 로드된 대응점 수: " << points.size() << std::endl;
    return points;
}

void PointLoader::SplitPoints(const std::vector<PointPair>& allPoints,
    std::vector<PointPair>& modelPoints,
    std::vector<PointPair>& testPoints,
    double modelRatio) {
    // 무작위 셔플 추가
    std::vector<PointPair> shuffled = allPoints;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(shuffled.begin(), shuffled.end(), gen);

    size_t modelSize = static_cast<size_t>(shuffled.size() * modelRatio);
    modelPoints.assign(shuffled.begin(), shuffled.begin() + modelSize);
    testPoints.assign(shuffled.begin() + modelSize, shuffled.end());

    std::cout << "[Info] 모델점 수: " << modelPoints.size()
        << " / 검사점 수: " << testPoints.size() << std::endl;
}
