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
        self.auto_manu = 1
        self.speed_scale =1. 
        self.redline_counter = 0 
        self.redline_locker = 0. 

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
        print("stop")
        cmd = f"stop:\n"
        self.ser.write(cmd.encode())
        self.angle = 0
        self.send_angle(self.angle)
        self.auto_manu = 0

    def send_forward(self,speed):
        cmd = f"forward:{int(speed*self.speed_scale)}\n"
        self.ser.write(cmd.encode())

    def send_backward(self,speed):
        cmd = f"backward:{int(speed*self.speed_scale)}\n"
        self.ser.write(cmd.encode())

    def send_angle(self,angle):
        cmd = f"angle:{angle}\n"
        self.ser.write(cmd.encode())

    def camera_callback(self, msg):
        if self.auto_manu ==1:
            self.camera_msg = json.loads(msg.data)
            if self.camera_msg["u1"]!="-1":
                x1=float(self.camera_msg["x1"])
                y1=float(self.camera_msg["y1"])
                self.angle = 90. - math.atan2(y1,x1) / math.pi * 180.
                #self.vitesse = float(self.camera_msg["h1"]) / 480. * self.vmax
                self.vitesse = self.vmax
                self.vitesse = 5. 
                #print(x1,y1)
                print("angle",self.angle,"vitesse",self.vitesse)
                if self.angle < -80. or self.angle > 80.:
                    print(msg)
                    quit()
                self.send_angle(self.angle)
                self.send_forward(self.vitesse)
            else:
                #self.vitesse=self.max
                #self.angle = 0
                print("no detection")
            if self.redline_locker==0 and "redline" in self.camera_msg:
                print("redline")
                self.redline_counter += 1 
                self.redline_locker = 1 
                threading.Timer(10, self.free_redline_locker, ()).start()

    def free_redline_locker(self):
        print("free redline locker")
        self.redline_locker = 0
        if self.redline_counter == 5:
            self.send_stop()

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
