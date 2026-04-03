#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

cv::Mat K = (cv::Mat1d(3, 3) << 
    498.044948695394, 0, 317.8820496352526,
    0, 498.789990131052, 236.7252253261842,
    0, 0, 1);
cv::Mat D = (cv::Mat1d(1, 5) <<
   -0.1734930987187489,
   -0.3795271453276246,
    0.0008427346618610709,
    0.0002321661103658256,
    0.1729582341969753);

double markerLength = 0.04; // 4cm

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::detectMarkers(gray, dictionary, corners, ids);

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, markerLength, K, D, rvecs, tvecs);

            for (size_t i = 0; i < ids.size(); ++i) {
                if (ids[i] == 1) {
                    // 마커 중심점 (마커 좌표계 기준 (0, 0, 0))
                    std::vector<cv::Point3f> objPoints = { cv::Point3f(0, 0, 0) };
                    std::vector<cv::Point2f> imgPoints;
                    cv::projectPoints(objPoints, rvecs[i], tvecs[i], K, D, imgPoints);

                    float cx = imgPoints[0].x;
                    float cy = imgPoints[0].y;

                    ROS_INFO_STREAM("마커 ID 1 중심점 픽셀 위치: (" << cx << ", " << cy << ")");
                    ROS_INFO_STREAM("마커 ID 1 카메라 좌표계 위치: x=" << tvecs[i][0]
                                    << ", y=" << tvecs[i][1]
                                    << ", z=" << tvecs[i][2]);

                    // 시각화
                    cv::circle(frame, imgPoints[0], 5, cv::Scalar(0, 255, 0), -1);
                    cv::aruco::drawAxis(frame, K, D, rvecs[i], tvecs[i], 0.03);
                }
            }
        }

        cv::imshow("Aruco Tracker", frame);
        cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_tracker_cpp");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);

    ROS_INFO("Aruco tracker (C++) node started");
    ros::spin();
    return 0;
}

