#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo
import numpy as np

def publish_camera_info():
    rospy.init_node('camera_info_publisher')

    pub = rospy.Publisher('/camera/image_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    # === 아래 값은 네가 캘리브레이션에서 얻은 값으로 바꾸면 됨 ===
    width = 640
    height = 480

    # 내부 파라미터 K
    K = [498.044948695394, 0.0, 317.8820496352526,
  0.0, 498.789990131052, 236.7252253261842,
  0.0, 0.0, 1.0]
    # 왜곡 계수 D
    D = [0.1743390075817489, 
         -0.3795271453276246, 
         0.0008427346618610709, 
         0.0002321661103658256, 
         0.1729582341969753]

    P = [K[0], 0.0, K[2], 0.0,
         0.0, K[4], K[5], 0.0,
         0.0, 0.0, 1.0, 0.0]

    while not rospy.is_shutdown():
        msg = CameraInfo()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera"
        msg.height = height
        msg.width = width
        msg.distortion_model = "plumb_bob"
        msg.D = D
        msg.K = K
        msg.R = np.identity(3).flatten().tolist()
        msg.P = P

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass

