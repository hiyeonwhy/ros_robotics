#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

int main() {
    // 예시: LiDAR 3D 점들 (Z=0인 2D LiDAR)
    std::vector<cv::Point3f> lidarPoints = {
        {0.248768, 0.275237, 0},
        {0.391173, 0.185669, 0},
        {1.237631, 0.160540, 0},
        {1.231193, -0.066738, 0}
    };

    // 내부 파라미터 (fx, fy, cx, cy)
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        498.0449486959394, 0, 317.8820496352526,
        0, 498.7899990131052, 236.7252253561842,
        0,     0,     1);

    // 외부 파라미터 (Rotation R, Translation t)
    cv::Mat R = (cv::Mat_<double>(3,3) <<
        0.3083, -0.9499, -0.0509,
        0.0925, 0.0832, -0.9922,
        0.9468, 0.3012, 0.1135
    );
    cv::Mat t = (cv::Mat_<double>(3,1) <<
        0.0158,
       -0.0748,
       -0.0424
    );

    // 이미지 로드 (점 찍기 위해)
    cv::Mat image = cv::imread("/home/hiyeon/ros_ws/src/camera_calibration_node/cali/calib0000.jpg");
    if (image.empty()) {
        std::cerr << "❌ 이미지 로드 실패!" << std::endl;
        return -1;
    }

    for (const auto& pt : lidarPoints) {
        cv::Mat X_lidar = (cv::Mat_<double>(3,1) << pt.x, pt.y, pt.z);

        // 카메라 좌표계로 변환: X_cam = R * X_lidar + t
        cv::Mat X_cam = R * X_lidar + t;
        double X = X_cam.at<double>(0);
        double Y = X_cam.at<double>(1);
        double Z = X_cam.at<double>(2);

        // Z < 0이면 카메라 뒤에 있는 점 → 무시
        if (Z <= 0) continue;

        // Projection: [u; v] = K * [X; Y; Z]
        double u = (K.at<double>(0,0) * X / Z) + K.at<double>(0,2);
        double v = (K.at<double>(1,1) * Y / Z) + K.at<double>(1,2);

        // 이미지에 점 찍기
        cv::circle(image, cv::Point(u, v), 4, cv::Scalar(0, 0, 255), -1);
        std::cout << "Projected point → u: " << u << ", v: " << v << std::endl;
    }

    // 결과 보기 및 저장
    cv::imshow("Projection", image);
    cv::imwrite("projected_result.jpg", image);
    cv::waitKey(0);

    return 0;
}

