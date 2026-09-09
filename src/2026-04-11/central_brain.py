import threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import json
import math

class CentralBrain(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('central_brain')
        self.subscriptionWeb = self.create_subscription(String,'web_command',self.web_callback, 10)
        self.subscriptionWeb
        self.subscriptionTopic = self.create_subscription(String,'camera_analysis',self.camera_callback, 10)
        self.subscriptionTopic

        self.vmax = 0.
        self.vitesse = 0.
        self.on_off = 0
        self.angle = 0.
        self.auto_manu = 0
        self.speed_scale = 1

        self.last_tick = 0
        self.tick = 0
        self.delta_time = ""
        self.buffer = 0
        self.camera_msg = {} 
        self.web_msg = ""

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        except:
            self.ser = 0

    def send_stop(self):
        self.vmax = 0.
        self.vitesse = 0.
        cmd = f"stop:\n"
        self.ser.write(cmd.encode())
        self.angle = 0
        self.send_angle(self.angle)

    def send_forward(self,speed):
        cmd = f"forward:{int(speed/self.speed_scale)}\n"
        self.ser.write(cmd.encode())

    def send_backward(self,speed):
        cmd = f"backward:{int(speed/self.speed_scale)}\n"
        self.ser.write(cmd.encode())

    def send_angle(self,angle):
        cmd = f"angle:{-angle}\n"
        self.ser.write(cmd.encode())

    def camera_callback(self, msg):
        if self.auto_manu ==1:
            print(msg.data)
            self.camera_msg = json.loads(msg.data)
            if self.camera_msg["m1"]==-1:
                self.vitesse=self.max
                self.angle = 0
                print("no detection")
            else:
                x1=float(self.camera_msg["x1"])
                y1=float(self.camera_msg["y1"])
                self.angle = math.atan2(x1,y1) / math.pi * 180.
                #self.vitesse = float(self.camera_msg["h1"]) / 480. * self.vmax
                self.vitesse = self.vmax
                print(x1,y1)
                print("angle",self.angle,"vitesse",self.vitesse)
                self.send_angle(self.angle)
                self.send_forward(self.vitesse)
        
    def web_callback(self, msg):
        print(msg.data)
        if msg.data == "mode_auto":
            self.auto_manu = 1 
            print("mode auto")
        if msg.data == "mode_manu":
            self.auto_manu = 0 
            print("mode manu")
        if msg.data in ["engine_on","engine_off"]:
            self.vmax = 0.
            self.send_stop()
        if msg.data[:4] == "vmax":
            self.vmax = int(msg.data.split(":")[1]) 
        if "manu_angle" in msg.data:
            self.angle = float(msg.data.split(":")[1]) 
            self.send_angle(self.angle)
        if "manu_speed_forward" in msg.data:
            self.vitesse = float(msg.data.split(":")[1]) 
            self.send_forward(self.vitesse)
        if "manu_speed_backward" in msg.data:
            self.vitesse = float(msg.data.split(":")[1]) 
            self.send_backward(self.vitesse)
        
def main(args=None):
    system = CentralBrain()
    rclpy.spin(system)
    system.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
