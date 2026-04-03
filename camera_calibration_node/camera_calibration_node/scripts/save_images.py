#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# 저장 폴더
save_dir = '/home/hiyeon/ros_ws/src/camera_calibration_node/cali'
os.makedirs(save_dir, exist_ok=True)

bridge = CvBridge()
count = 0

def image_callback(msg):
    global count
    try:
        # ROS 이미지 → OpenCV 이미지
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 이미지 보여주기
        cv2.imshow("Camera View (Press SPACE to save)", cv_image)
        key = cv2.waitKey(1)

        if key == 32:  # 스페이스바 입력 시 저장
            filename = os.path.join(save_dir, f'calib{count:04d}.jpg')
            cv2.imwrite(filename, cv_image)
            print(f"✅ Saved: {filename}")
            count += 1
        elif key == 27:  # ESC 입력 시 종료
            rospy.signal_shutdown("User exit")
            cv2.destroyAllWindows()

    except Exception as e:
        rospy.logerr(f"Image conversion error: {e}")

def main():
    rospy.init_node('image_capture_node')
    rospy.Subscriber('/camera/image', Image, image_callback)
    print("📷 Press SPACE to save image, ESC to quit.")
    rospy.spin()

if __name__ == '__main__':
    main()

