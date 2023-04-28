#!/usr/bin/env python3
import time
import socket
import threading
import rospy
from sensor_msgs.msg import Image


class Image_Bridge:
    def __init__(self, ip, port, data_step, sleepTime=0.1):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udpInfo = (ip, port)
        self.image = None
        self.data_step = data_step
        self.sleepTime = sleepTime

    def callback(self, msg):
        self.image = msg

    def publish(self):
        while True:
            if self.image == None:
                continue
            info = [
                self.image.width,
                self.image.height,
                self.image.step,
                self.data_step,
            ]
            send_str = " ".join(str(s) for s in info)
            self.sock.sendto(send_str.encode(), self.udpInfo)
            for i in range(0, self.image.height, self.data_step):
                start = i * self.image.step
                self.sock.sendto(
                    self.image.data[start : (start + self.data_step * self.image.step)],
                    self.udpInfo,
                )
            self.image = None
            time.sleep(self.sleepTime)

    def run(self):
        rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        server_thread = threading.Thread(target=self.publish)
        server_thread.daemon = True
        server_thread.start()


if __name__ == "__main__":
    rospy.init_node("UDP_Bridge_Image", anonymous=True)
    image_bridge = Image_Bridge(
        rospy.get_param("~udp_ip"),
        rospy.get_param("~udp_image_port"),
        rospy.get_param("~image_data_step"),
        rospy.get_param("~udp_sleep_time"),
    )
    image_bridge.run()
    rospy.spin()
