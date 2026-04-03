#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace cv;

int main() {
    Size boardSize(8, 6);  // 내부 코너 수

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;

    // 3D 체커보드 점 생성
    vector<Point3f> objp;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            objp.push_back(Point3f(j, i, 0));
        }
    }

    // 이미지 이름: calib0000.jpg ~ calib0025.jpg
    for (int i = 0; i <= 25; i++) {
        stringstream ss;
        ss << "/home/hiyeon/ros_ws/src/camera_calibration_node/cali/calib"  // 경로 포함!
           << setw(4) << setfill('0') << i << ".jpg";
        string filename = ss.str();

        Mat img = imread(filename);
        if (img.empty()) {
            cout << "❌ 이미지를 불러올 수 없습니다: " << filename << endl;
            continue;
        }

        vector<Point2f> corners;
        bool found = findChessboardCorners(img, boardSize, corners);

        if (found) {
            Mat gray;
            cvtColor(img, gray, COLOR_BGR2GRAY);
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

            imagePoints.push_back(corners);
            objectPoints.push_back(objp);

            drawChessboardCorners(img, boardSize, corners, found);
            imshow("Chessboard", img);
            waitKey(100);
        } else {
            cout << "⚠️ 체커보드를 찾지 못했습니다: " << filename << endl;
        }
    }

    destroyAllWindows();

    if (imagePoints.empty()) {
        cerr << "❗ 유효한 체커보드 이미지가 없어 캘리브레이션을 수행할 수 없습니다." << endl;
        return -1;
    }

    // 내부 파라미터 계산
    Mat cameraMatrix, distCoeffs, R, T;
    calibrateCamera(objectPoints, imagePoints, Size(640, 480), cameraMatrix, distCoeffs, R, T);

    cout << "\n📷 Camera Matrix:\n" << cameraMatrix << endl;
    cout << "\n🎯 Distortion Coefficients:\n" << distCoeffs << endl;

    return 0;
}

