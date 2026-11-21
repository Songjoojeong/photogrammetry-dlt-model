# photogrammetry-dlt-model

인하대학교 사진측량학 Mid-Term Project (Topic 7) 코드입니다.
항공영상(인하대 일대)을 대상으로 2D 변환 모델, DLT, 정밀 센서 모델의 정합 정확도를 RMSE로 비교했습니다.
코드는 최종 보고서 부록(6.2 실험 코드)과 같은 버전입니다.

## 구성

| 폴더 | 내용 | 입력 | 라이브러리 |
|---|---|---|---|
| `Part1_PointTransformation` | Affine / Homography 변환 (최소제곱) | `points_roof.txt`, `points_ground.txt` (R0047629 ↔ R0047630 대응점) | Eigen |
| `Part2_DLT_Calibration` | 직접 DLT (SVD) / 간접 DLT (IOP 경유) | 코드 내 GCP 12개 (모델 6, 검사 6) | OpenCV |
| `Part3_SensorRefinement` | 공선조건식 기반 정밀 센서 모델, Gauss–Newton EOP 보정 | 코드 내 GCP 12개 (모델 5, 검사 7), IOP, 초기 EOP | OpenCV |

대응점 파일 형식은 한 줄에 `x1,y1,x2,y2`(영상1 열·행, 영상2 열·행)입니다. 지붕 12쌍, 바닥 16쌍입니다.

## 결과 (보고서 기준, 단위 px)

| 모델 | 모델점 RMSE | 검사점 RMSE |
|---|---|---|
| Affine (지붕) | 5.9173 | 9.9219 |
| Homography (지붕) | 1.7397 | 9.9882 |
| Affine (바닥) | 9.6606 | 9.2500 |
| Homography (바닥) | 1.9894 | 3.5389 |
| Affine (전체) | 11.4547 | 23.7636 |
| Homography (전체) | 11.1967 | 22.0626 |
| Direct DLT | – | 11.0107 |
| Indirect DLT | – | 11.0949 |
| Precision Sensor Model (보정 전) | 42.1159 | 36.2055 |
| Precision Sensor Model (보정 후) | 3.8168 | 4.6353 |

각 폴더의 `output.txt`에 실행 결과가 있습니다. Part 2와 Part 3은 결정적이라 다시 실행해도 같은 값이 나옵니다.
Part 1은 `std::random_device`로 모델점/검사점을 무작위 분할하므로 실행할 때마다 값이 달라지며, `output.txt`는 보고서 제출 당시 실행 결과입니다.

## 빌드

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
cd build
./part1_transform   # points_*.txt는 build 폴더로 자동 복사됨
./part2_dlt
./part3_sensor
```

필요 환경: C++17, OpenCV 4.x, Eigen 3 (보고서 작성 환경은 OpenCV 4.5.1, Visual Studio).

## 참고

- Part 2 코드의 `targetImage` 문자열은 `R0047627.JPG`로 되어 있으나 식별용 이름일 뿐 계산에는 쓰이지 않습니다. GCP 영상좌표는 보고서 본문 기준 R0047632.JPG에서 측정한 값입니다.
- Part 3의 회전행렬은 `R = Rκ · Rφ · Rω` 순서이며, 주점은 영상 중심(2464, 1632) px로 두었습니다.
