#!/usr/bin/env python3
import socket
import threading
import rospy
from sensor_msgs.msg import Image


class Image_Bridge():
    def __init__(self,ip,port):
        self.sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udpInfo = (ip,port)
        self.image = None

    def callback(self,msg):
        self.image = msg
  
    def publish(self):
        while True:
            if(self.image == None):
                continue
            info = [self.image.width, self.image.height, self.image.step]
            send_str = " ".join(str(s) for s in info)
            self.sock.sendto(send_str.encode(), self.udpInfo)
            for i in range(0, self.image.height, 65):
                self.sock.sendto(self.image.data[i*self.image.step:(i*self.image.step+65*self.image.step)], self.udpInfo)
            self.image = None

    def run(self):
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback, tcp_nodelay=True)
        server_thread = threading.Thread(target=self.publish)
        server_thread.daemon = True
        server_thread.start()




if __name__ == "__main__":
    rospy.init_node('UDP_Bridge_Image', anonymous=True)
    image_bridge = Image_Bridge(rospy.get_param("~udp_ip"), rospy.get_param("~udp_image_port"))
    image_bridge.run()
    rospy.spin()