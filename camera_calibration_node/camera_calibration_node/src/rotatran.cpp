#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
    // 3D LiDAR 좌표 (단위: meter)
    std::vector<cv::Point3f> objectPoints = {
        {0.248768, 0.275237, 0},
        {0.391173, 0.185669, 0},
        {1.237631, 0.160540, 0},
        {1.231193, -0.066738, 0}
    };

    // 이미지 상 2D 픽셀 좌표 (단위: pixel)
    std::vector<cv::Point2f> imagePoints = {
        {13, 186},
        {266, 204},
        {419, 258},
        {527, 254}
    };

    // 카메라 내부 파라미터 행렬 (fx, fy, cx, cy)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
                             498.0449486959394, 0, 317.8820496352526,
                             0, 498.7899990131052, 236.7252253561842,
                             0, 0, 1);

    // 왜곡 계수 (필요 없다면 0으로 설정)
    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64F);

    // 출력값: 회전 벡터, 이동 벡터
    cv::Mat rvec, tvec;

    // solvePnP
    bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

    if (!success) {
        std::cerr << "❌ solvePnP failed!" << std::endl;
        return -1;
    }

    // 회전 행렬로 변환
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 출력
    std::cout << "✅ Rotation Matrix (R):\n" << R << std::endl;
    std::cout << "✅ Translation Vector (t):\n" << tvec << std::endl;

    return 0;
}

