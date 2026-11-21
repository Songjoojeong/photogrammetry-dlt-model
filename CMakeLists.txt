cmake_minimum_required(VERSION 3.10)
project(photogrammetry_midterm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Part 1 - Affine / Homography (Eigen)
add_executable(part1_transform
    Part1_PointTransformation/Main01.cpp
    Part1_PointTransformation/PointLoader.cpp
    Part1_PointTransformation/TransformModel.cpp)
target_link_libraries(part1_transform Eigen3::Eigen)
# 대응점 파일을 실행 위치로 복사
configure_file(Part1_PointTransformation/points_roof.txt   ${CMAKE_CURRENT_BINARY_DIR}/points_roof.txt   COPYONLY)
configure_file(Part1_PointTransformation/points_ground.txt ${CMAKE_CURRENT_BINARY_DIR}/points_ground.txt COPYONLY)

# Part 2 - Direct / Indirect DLT (OpenCV)
add_executable(part2_dlt Part2_DLT_Calibration/Main2.cpp)
target_include_directories(part2_dlt PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(part2_dlt ${OpenCV_LIBS})

# Part 3 - Precision Sensor Model (OpenCV)
add_executable(part3_sensor Part3_SensorRefinement/FrameCamera.cpp)
target_include_directories(part3_sensor PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(part3_sensor ${OpenCV_LIBS})
