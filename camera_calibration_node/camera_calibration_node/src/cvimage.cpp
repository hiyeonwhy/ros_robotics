#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main() {
    // 🔹 내부 파라미터와 왜곡 계수
    Mat cameraMatrix = (Mat_<double>(3,3) << 
        498.0449486959394, 0, 317.8820496352526,
        0, 498.7899990131052, 236.7252253261842,
        0, 0, 1);
    Mat distCoeffs = (Mat_<double>(1,5) << 
        0.1743390075817489, -0.3795271453276246,
        0.0008427364618610709, 0.0002321661103658256,
        0.1729582341969753);

    // 🔹 이미지 불러오기
    Mat img = imread("/home/hiyeon/ros_ws/src/camera_calibration_node/cali/calib0022.jpg");
    if (img.empty()) {
        cout << "이미지를 불러올 수 없습니다." << endl;
        return -1;
    }

    // 🔹 보정 행렬 계산 및 보정
    Mat newCameraMatrix;
    Rect roi;
    newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, img.size(), 1, img.size(), &roi);

    Mat undistorted;
    undistort(img, undistorted, cameraMatrix, distCoeffs, newCameraMatrix);

    // 🔹 ROI 영역으로 잘라서 사용 가능하게 만들기
    Mat cropped = undistorted(roi);

    // 🔹 결과 출력
    imshow("Original", img);
    imshow("Undistorted Cropped", cropped);
    waitKey(0);

    // 🔹 저장
    imwrite("undistorted_cropped.jpg", cropped);

    return 0;
}

