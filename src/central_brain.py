import threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import ReliabilityPolicy

from std_msgs.msg import String
import serial
import time
import json
import math
import struct

class CentralBrain(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('central_brain')

        qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )

        self.subscriptionWeb = self.create_subscription(String,'web_command',self.web_callback, 10)
        self.subscriptionWeb
        self.subscriptionTopic = self.create_subscription(String,'camera_analysis',self.camera_callback, qos)
        self.subscriptionTopic
        self.publisher_brain = self.create_publisher(String, 'brain_status', 10)

        self.brain_status = {}
        self.vmax_init = 15.
        self.vmax = self.vmax_init 
        self.vitesse = 0.
        self.on_off = 0
        self.angle = 0.
        self.auto_manu = 1
        self.redline_counter = 0 
        self.redline_locker = 0. 

        self.odometre = 0

        self.last_tick = 0
        self.tick = 0
        self.delta_time = ""
        self.buffer = 0
        self.camera_msg = {} 
        self.web_msg = ""

        self.esp32_FMT = "<IIllffii"
        self.esp32_msg_size = struct.calcsize(self.esp32_FMT)

        self.data_esp32={}

        self.to_esp32_FMT = "<Hh"
        self.to_esp32_msg_size = struct.calcsize(self.to_esp32_FMT)
        try:
            self.ser = serial.Serial('/dev/ttyUSB0' , baudrate=115200, timeout=0, write_timeout=0) 
        except:
            self.ser = 0
        threading.Thread(target=self.read_serial, daemon=True).start()

    def read_serial(self):
        while True:
             if self.ser.in_waiting >= self.esp32_msg_size:
                data = self.ser.read(self.esp32_msg_size)
                if len(data) == self.esp32_msg_size:
                    self.data_esp32={}
                    (
                        self.data_esp32["time_esp32"],
                        self.data_esp32["last_time_esp32"],
                        self.data_esp32["count"],
                        self.data_esp32["last_count"],
                        self.data_esp32["current_rpm"],
                        self.data_esp32["target_rpm"],
                        self.data_esp32["last_pwm"],
                        self.data_esp32["current_pwm"]
                    ) = struct.unpack(self.esp32_FMT,data)
                    self.odometre = self.data_esp32["count"]
                    self.data_esp32["delta_time_esp32"]=self.data_esp32["time_esp32"]-self.data_esp32["last_time_esp32"]
                    self.data_esp32["delta_count"]=self.data_esp32["count"]-self.data_esp32["count"]
                    if self.data_esp32["delta_time_esp32"] >0:
                        self.data_esp32["speed_esp32"]=self.data_esp32["delta_count"]/self.data_esp32["delta_time_esp32"]
                    else:
                        self.data_esp32["speed_esp32"]=-1.

    def publish_brain_status(self):
        msg = String()
        msg.data=json.dumps(self.brain_status)
        self.publisher_brain.publish(msg)

    def send_stop(self):
        self.vmax = 0.
        self.vitesse = 0.
        self.on_off = 0
        self.auto_manu = 0
        print("stop")
        cmd = f"s:\n"
        self.ser.write(cmd.encode())
        #data = struct.pack('<Hh', 0, 0)
        #self.ser.write(data)
        self.send_angle(0)

    def send_forward(self,speed):
        cmd = f"f:{int(speed)}\n"
        self.ser.write(cmd.encode())
        #data = struct.pack('<Hh', 1, int(speed))
        #self.ser.write(data)

    def send_backward(self,speed):
        cmd = f"b:{int(speed)}\n"
        self.ser.write(cmd.encode())
        #data = struct.pack('<Hh', 2, int(speed))
        #self.ser.write(data)

    def send_angle(self,angle):
        cmd = f"a:{angle}\n"
        self.ser.write(cmd.encode())
        #data = struct.pack('<Hh', 3, int(angle))
        #self.ser.write(data)

    def camera_callback(self, msg):
        if self.on_off == 1:
            self.camera_msg = json.loads(msg.data)
            if self.camera_msg["u1"]!="-1":
                x1=float(self.camera_msg["x1"])
                y1=float(self.camera_msg["y1"])
                self.angle = 90. - math.atan2(y1,x1) / math.pi * 180.
                print(time.time())
                print(msg.data)
                print("angle",self.angle,"vitesse",self.vitesse)
                if self.angle < -80. or self.angle > 80.:
                    print(msg)
                    return -1
                self.send_angle(self.angle)
                if self.odometre > 28000: 
                    self.send_stop()
                if self.odometre == 0: 
                    self.send_forward(self.vitesse)
                self.brain_status={}
                self.brain_status = self.camera_msg.copy()
                self.brain_status.update(self.data_esp32)
                self.brain_status["camera_order_time"]=self.camera_msg["time"]
                self.brain_status["brain_time"]=time.time()
                self.brain_status["delta_cam_brain_time"]=time.time()-self.camera_msg["time"]
                self.brain_status["angle"]=self.angle
                self.brain_status["odometre"]=self.odometre
                self.brain_status["vitesse"]=self.vitesse
                self.publish_brain_status()
            if self.camera_msg["error"]!="-1":
                error=float(self.camera_msg["error"])
                #self.vitesse = float(self.camera_msg["h1"]) / 480. * self.vmax
                self.vitesse = self.vmax
            if "redline" in self.camera_msg:
                print("redline")
                self.redline_counter += 1 
                if self.redline_counter == 6:
                    self.send_stop()

    def web_callback(self, msg):
        print(msg.data)
        if msg.data == "mode_auto":
            self.auto_manu = 1 
            print("mode auto")
        if msg.data == "mode_manu":
            self.auto_manu = 0 
            self.vmax = 0.
            self.send_stop()
            print("mode manu")
        if msg.data in ["engine_on"]:
            self.vmax = self.vmax_init 
            self.on_off = 1
        if msg.data in ["engine_off"]:
            self.send_stop()
            self.on_off = 0
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
    node = CentralBrain()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
