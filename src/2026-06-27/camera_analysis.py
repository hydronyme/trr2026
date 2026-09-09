import threading
from queue import Queue
import cv2 as cv
import numpy as np
from math import sqrt
import time
import os
import vpi
from PIL import Image
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# to define the camera parameters refer to : https://github.com/JetsonHacksNano/CSI-Camera
class MinimalPublisher(Node):
    def __init__(self):
        # Configuration

        self.WIDTH = 1280
        self.HEIGHT = 720
        # Calibration camera 1280 x 720
        self.u0=640
        self.beta_x = 0.13646387832699614
        self.vh = 200 
        self.beta_y = 67.19999999999997

        self.tick_frequency = cv.getTickFrequency() / 1000.
        self.font = cv.FONT_HERSHEY_PLAIN
        self.font_scale = 1.0
        
        # État du système
        self.report = {}
        self.result = {"u1": -1, "v1": 0, "u2": -1, "v2": 0}
        self.loop_counter = 0
        self.img_cam = np.zeros((self.HEIGHT, self.WIDTH, 3), np.uint8)
        self.img_canny = np.zeros((self.HEIGHT, self.WIDTH), np.uint8)
        
        self.start = cv.getTickCount()
        self.publish_start = cv.getTickCount()

        # Paramètres OpenCV
        self.param_canny_min = 150
        self.param_canny_max = 200
        cv.setUseOptimized(True)
        self.br = CvBridge()

        # Initialisation matériel
        self.camera = self._set_camera()
        self.output = []

        self.red_line_time = cv.getTickCount()


        super().__init__('minimal_publisher')
        self.publisher_img_cam = self.create_publisher(Image, 'video_frames', 1)
        self.publisher_report = self.create_publisher(String, 'camera_analysis', 10)
        self.subscriptionWeb = self.create_subscription(String,'web_command',self.web_callback, 10)
        self.subscriptionWeb

        self.lock = threading.Lock()
        self.queue = Queue(maxsize=1)   # 👈 IMPORTANT
        threading.Thread(target=self.update, daemon=True).start()
        threading.Thread(target=self.infinite_loop, daemon=True).start()

        print("init ok")

    def web_callback(self, msg):
        if msg.data == "save_img":
            filename = "/home/montaulab/Pictures/cam_" + str(cv.getTickCount()) + ".jpg" 
            cv.imwrite(filename, self.img_cam)

    def update(self):
        while True:
            ret, frame = self.camera.read()
            if ret:
                with self.lock:
                    self.img_cam = frame
                    self.queue.put(frame)

    @staticmethod
    def chronometre(func):
        def wrapper(self, *args, **kwargs):
            start = cv.getTickCount()
            result = func(self,*args, **kwargs)
            end = cv.getTickCount()
            if self.loop_counter >= 20:
                t = (end - start) / self.tick_frequency
                key = "F_" + func.__name__
                self.report[key] = t
                self.report[key + "_sum"] = self.report.get(key + "_sum", 0) + t
                self.report[key + "_count"] = self.report.get(key + "_count", 0) + 1
            return result
        return wrapper

    # --- Initialisation ---
    def _set_camera(self):

        gstreamer = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)60/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        )

        cam = cv.VideoCapture(gstreamer, cv.CAP_GSTREAMER)
        print(cam)
        #time.sleep(10)

        return cam


    #@chronometre
    def find_redline(self):
        if (cv.getTickCount() - self.red_line_time) / self.tick_frequency < 10000.: 
            return 0

        img2=cv.resize(self.img_cam, (self.WIDTH//4, self.HEIGHT//4), interpolation=cv.INTER_AREA)
        hsv = cv.cvtColor(img2, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, (120, 123, 0), (240, 255, 255))
        row_density = mask[::10].sum(axis=1)
        col_density = mask[:,::10].sum(axis=0)
        y = np.where(row_density>200)[0]
        x = np.where(col_density>50)[0]
        if len(x) and len(y):
            x1, x2 = x[0]*40, x[-1]*40
            y1, y2 = y[0]*40, y[-1]*40
            v0 = (self.vh + self.HEIGHT) //2
            if y1> v0 and y2 - y1 > 50 and x2 - x1 > 200:
                cv.line(self.img_cam, (x1, y1), (x2, y2), (255,255,0), 3)
                self.result["redline"]=1
                print(">>>>redline found")
                self.red_line_time = cv.getTickCount()
                return 1
        return 0

    def analyse_line(self,gray, canny, v):
        edges = canny[v:v+1, :].ravel()
        color = gray[v:v+1, :].ravel()
        bord = np.where(edges == 255)[0]
        if bord.shape[0]<3: return -1,0 # we want to have at least 4 edges.
        bord = bord[np.ediff1d(bord,to_end=20) > 9] # keeps only the large difference
        delta = np.ediff1d(bord)
        mean_color=np.empty(len(bord)-1,dtype=int)
        for idx,b in enumerate(bord[:-1]):
            mean_color[idx]=int(np.mean(color[b:bord[idx+1]]))

        for idx in range(1,len(mean_color)-1):
            if mean_color[idx] > 200 and mean_color[idx-1]<80 and mean_color[idx+1]<80 and abs(delta[idx]-delta[idx-1])<delta[idx]/2 and abs(delta[idx]-delta[idx+1])<delta[idx]/2:
                u=(bord[idx]+bord[idx+1]) // 2
                x = self.beta_x * (u -self.u0) / (v - self.vh)
                return (u,x)
        return -1,0

    #@chronometre
    def find_band(self, gray, canny):
        k = 0
        v0 = (self.vh + self.HEIGHT) //2
        while True:
            v = v0 + k * 10
            if v >= self.HEIGHT:
                return -1, 0, 0, 0
            u, x = self.analyse_line(gray, canny, v)
            if u != -1:
                y = self.beta_y / (v - self.vh)
                self.output.append({"u":u,"v":v,"x":x,"y":y})
                return u, v, x, y
            k += 1
            v = v0 - k * 10
            if v <= self.vh:
                return -1, 0, 0, 0
            u, x = self.analyse_line(gray, canny, v)
            if u != -1:
                y = self.beta_y / (v - self.vh)
                self.output.append({"u":u,"v":v,"x":x,"y":y})
                return u, v, x, y

    #@chronometre
    def read_camera(self):
        return self.camera.read()

    # --- Boucle Principale (Thread) ---
    def infinite_loop(self):
        while rclpy.ok():
            self.loop_item()

    #@chronometre
    def _prepare_image_internal(self, img):
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        blur= cv.GaussianBlur(gray, (5, 5), 0)
        eq = cv.equalizeHist(blur)
        return eq

    #@chronometre
    def loop_item(self):

        self.loop_counter += 1
        frame = self.queue.get()
        self.result = {}

        prepare_start = cv.getTickCount() 
        t1 = time.time()
        gray = self._prepare_image_internal(frame)
        self.img_canny = cv.Canny(gray, self.param_canny_min, self.param_canny_max)
        prepare_end = cv.getTickCount() 
        #print("delta prepare t",(prepare_end-prepare_start) / self.tick_frequency)

        self.output = []
        self.result["u1"],self.result["v1"],self.result["x1"], self.result["y1"] = self.find_band(gray, self.img_canny)
        #if self.loop_counter % 10 ==0:
        #   self.find_redline()
        """
        if len(self.output)>0:
            self.result["u1"]=self.output[0]["u"]
            self.result["v1"]=self.output[0]["v"]
            self.result["x1"]=self.output[0]["x"]
            self.result["y1"]=self.output[0]["y"]
        #self.result["u2"],self.result["v2"],self.result["x2"], self.result["y2"] = self.find_band(gray, canny, -1)
        """
        self.publish_text_report()
        #if self.loop_counter % 100 ==0:
        #self.publish_img_cam(frame)
        end = cv.getTickCount()
        t2 = time.time()
        t = (end - prepare_start) / self.tick_frequency
        #print("delta loop t",t)
        #print("delta time",t2-t1)
        t = (end - self.start) / self.tick_frequency
        #print("delta t",t)
        self.start = end
        self.queue.task_done()

    def publish_img_cam(self,frame):
        for o in self.output:
            if "u" in o and "v" in o:
                cv.circle(frame, (o["u"], o["v"]), 5, (255, 0, 0), -1)
        self.publisher_img_cam.publish(self.br.cv2_to_imgmsg(frame))

    def publish_text_report(self):
        msg = String()
        x= {} 
        for k, v in self.result.items():
            x[k]=str(v)
        for k, v in self.report.items():
            if k[-4:]=="_sum":
                x[k[:-4]+"_mean"] =  str(v / self.report[k[:-4]+"_count"])
            elif k[-6:]=="_count":
                continue
            else:
                x[k]=str(v)
        x["time"]=time.time()
        end = cv.getTickCount()
        x["delta_publish"]= (end - self.publish_start) / self.tick_frequency
        #print("delta publish ",t)
        msg.data=json.dumps(x)
        self.publish_start = cv.getTickCount()
        if x["u1"]=="-1":
            print("no publication",msg.data)
        else:
            print("publish",msg.data)
        self.publisher_report.publish(msg)

    def run_ros(self):
        node = self
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = MinimalPublisher()

    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

