#!/usr/bin/env python3
import time
import socket
import threading
import rospy
from sensor_msgs.msg import PointCloud2


class PointCloud2_Bridge:
    def __init__(self, ip, port, data_step, sleepTime=0.1):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udpInfo = (ip, port)
        self.pointcloud2 = None
        self.data_step = data_step
        self.sleepTime = sleepTime

    def callback(self, msg):
        self.pointcloud2 = msg

    def publish(self):
        while True:
            if self.pointcloud2 == None:
                continue
            info = [
                self.pointcloud2.width,
                self.pointcloud2.height,
                self.pointcloud2.row_step,
                self.pointcloud2.point_step,
                self.data_step,
            ]
            send_str = " ".join(str(s) for s in info)
            self.sock.sendto(send_str.encode(), self.udpInfo)
            for i in range(0, self.pointcloud2.height, self.data_step):
                start = i * self.pointcloud2.row_step
                self.sock.sendto(
                    self.pointcloud2.data[
                        start : (start + self.data_step * self.pointcloud2.row_step)
                    ],
                    self.udpInfo,
                )
            self.pointcloud2 = None
            time.sleep(self.sleepTime)

    def run(self):
        rospy.Subscriber(
            "/camera/depth_registered/points",
            PointCloud2,
            self.callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        server_thread = threading.Thread(target=self.publish)
        server_thread.daemon = True
        server_thread.start()


if __name__ == "__main__":
    rospy.init_node("UDP_Bridge_PointCloud2", anonymous=True)
    pointcloud2_bridge = PointCloud2_Bridge(
        rospy.get_param("~udp_ip"),
        rospy.get_param("~udp_pointCloud2_port"),
        rospy.get_param("~pointCloud2_data_step"),
        rospy.get_param("~udp_sleep_time"),
    )
    pointcloud2_bridge.run()
    rospy.spin()
