#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>

// Camera calibration parameters
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
ros::Publisher cmd_pub;

// Docking parameters
const double theta_tol = 0.005;            // [rad] Acceptable angle error
const double z_target = 0.03;            // [m] Target docking distance (3cm 앞에서 멈춤)
const double z_tol = 0.008;               // [m] Acceptable z error
const double extra_straight = 0.18;       // [m] STRAIGHT-ONLY MODE에서 항상 추가로 더 가는 거리 (3cm)
const double initial_forward_distance = 0.6; // [m] First straight distance
const double rotate_speed = 0.2;         // [rad/s] Fast rotate when searching
const double align_rotate_speed = 0.08;   // [rad/s] Precise rotate when docking
const double approach_speed = 0.045;      // [m/s] Precise forward speed

const double z_switch = 0.07;             // [m] If closer than this, skip angle and move straight

geometry_msgs::Pose start_pose;
bool got_start_pose = false;

// STRAIGHT-ONLY MODE 변수들
static bool straight_only_mode = false;
static double straight_travel_distance = 0.0;
static geometry_msgs::Pose straight_start_pose;

double get_distance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

enum DockingState {
    INITIAL_FORWARD,   // First: Move forward by a fixed distance
    AUTO_MODE,         // After: Dock if marker found, else fast rotate
};
DockingState state = INITIAL_FORWARD;

void stopRobot() {
    geometry_msgs::Twist stop;
    cmd_pub.publish(stop);
}

// Final orientation correction (precise alignment)
void finalOrientationCorrection(double x, double z) {
    double theta = atan2(x, z);
    if (fabs(theta) > 0.005) {
        geometry_msgs::Twist rot;
        rot.angular.z = (theta > 0) ? -0.03 : 0.03;
        double duration = fabs(theta) / 0.03;
        ros::Rate rate(15);
        int ticks = duration * 15;
        ROS_INFO_STREAM("[Docking] Final orientation correction, theta=" << theta);
        for (int i = 0; i < ticks; ++i) {
            cmd_pub.publish(rot);
            rate.sleep();
        }
    }
    stopRobot();
    ROS_INFO("[Docking] Docking complete and orientation aligned!");
}

// Odometry callback for distance-based initial straight
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static bool docking_completed = false;

    // STRAIGHT-ONLY MODE: 오도메트리 기준 거리 도달 확인!
    if (straight_only_mode) {
        double traveled = get_distance(msg->pose.pose, straight_start_pose);
        if (traveled >= straight_travel_distance - z_tol) {
            stopRobot();
            ROS_INFO("[Docking] STRAIGHT-ONLY MODE done (odom). Docking complete!");
            straight_only_mode = false;
            docking_completed = true;
            ros::shutdown();
        } else {
            geometry_msgs::Twist cmd;
            cmd.linear.x = approach_speed;
            cmd.angular.z = 0.0;
            cmd_pub.publish(cmd);
            ROS_INFO_STREAM_THROTTLE(1, "[Docking] STRAIGHT-ONLY MODE (odom-based) traveling... " << traveled << " / " << straight_travel_distance);
        }
        return;
    }

    if (state != INITIAL_FORWARD) return;
    if (!got_start_pose) return;

    double dist = get_distance(msg->pose.pose, start_pose);
    if (dist >= initial_forward_distance) {
        stopRobot();
        state = AUTO_MODE;
        ROS_INFO("[Docking] Initial forward distance reached (%.2fm). Start AUTO_MODE.", dist);
    }
}

// Main docking logic (image callback)
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static bool docking_completed = false;
    static bool docking_stopped = false;
    static double last_z = 100.0;             // 마지막으로 본 z값

    if (docking_completed) return;
    if (straight_only_mode) return;

    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::detectMarkers(gray, dictionary, corners, ids);

        geometry_msgs::Twist cmd;
        bool marker_found = false;
        double x = 0, z = 0;

        // 1. Initial straight based on odometry
        if (state == INITIAL_FORWARD) {
            cmd.linear.x = 0.08;
            cmd.angular.z = 0.0;
            cmd_pub.publish(cmd);
            return;
        }

        // 2. Docking logic after initial straight
        if (state == AUTO_MODE) {
            if (!ids.empty()) {
                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, markerLength, K, D, rvecs, tvecs);

                for (size_t i = 0; i < ids.size(); ++i) {
                    if (ids[i] == 1) {
                        marker_found = true;
                        x = tvecs[i][0];
                        z = tvecs[i][2];
                        double theta = atan2(x, z);

                        // ---------- 핵심: STRAIGHT-ONLY MODE 진입 시 추가 직진 거리 더함 ----------
                        if (z < z_switch) {
                            straight_only_mode = true;
                            last_z = z;
                            // STRAIGHT-ONLY 모드용 시작점/이동거리 저장
                            ros::NodeHandle nh;
                            nav_msgs::Odometry::ConstPtr curr_odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nh, ros::Duration(0.5));
                            if (curr_odom) {
                                straight_start_pose = curr_odom->pose.pose;
                                straight_travel_distance = last_z - z_target + extra_straight;
                                ROS_INFO_STREAM("[Docking] Switched to STRAIGHT-ONLY MODE! (last_z=" << last_z
                                    << ", z_target=" << z_target
                                    << ", extra_straight=" << extra_straight
                                    << ", straight_travel_distance=" << straight_travel_distance << ")");
                            } else {
                                ROS_WARN("[Docking] Could not get current odometry for STRAIGHT-ONLY MODE. Fallback: do not enter straight-only mode.");
                                stopRobot();
                                docking_completed = true;
                                ros::shutdown();
                            }
                            return;
                        } else {
                            // 충분히 멀리 있으면 각도 정렬 후 직진
                            if (fabs(theta) > theta_tol) {
                                cmd.angular.z = (theta > 0) ? -align_rotate_speed : align_rotate_speed;
                                cmd.linear.x = 0.0;
                                ROS_INFO_STREAM("[Docking] Rotating to align angle (theta=" << theta << ")");
                                docking_stopped = false;
                            } else {
                                if (fabs(z - z_target) > z_tol) {
                                    cmd.angular.z = 0.0;
                                    cmd.linear.x = (z > z_target) ? approach_speed : -approach_speed;
                                    ROS_INFO_STREAM("[Docking] Moving forward to target z... z=" << z);
                                    docking_stopped = false;
                                } else {
                                    // 최종 각도 보정
                                    if (!docking_stopped) {
                                        finalOrientationCorrection(x, z);
                                        docking_stopped = true;
                                        stopRobot();
                                        docking_completed = true;
                                        ROS_INFO("[Docking] Docking fully complete! Node will now stop.");
                                        ros::shutdown();
                                    }
                                }
                            }
                        }
                        last_z = z;
                        break;
                    }
                }
            } 
            // Marker not found: fast rotate
            if (!marker_found) {
                cmd.linear.x = 0.0;
                cmd.angular.z = -rotate_speed;
                cmd_pub.publish(cmd);
                ROS_WARN_THROTTLE(2, "[Docking] Marker lost! Rotating CW to find marker (fast).");
            } else {
                if (!docking_stopped) {
                    cmd_pub.publish(cmd);
                }
            }
        }
    } catch (cv_bridge::Exception& e) {
        stopRobot();
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_distance_forward_docking");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);

    ROS_INFO("🚀 aruco_distance_forward_docking node started (distance-based initial forward, fast rotate search, slow docking, odom-based straight-only mode, extra forward applied)");

    // Save initial pose for odometry-based straight
    nav_msgs::Odometry::ConstPtr first_odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nh);
    if (first_odom) {
        start_pose = first_odom->pose.pose;
        got_start_pose = true;
        ROS_INFO("[Docking] Start pose recorded.");
    } else {
        ROS_WARN("[Docking] Failed to get initial pose!");
    }

    ros::spin();
    return 0;
}

