import threading
from queue import Queue
import cv2
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

        self.WIDTH = 640
        self.HEIGHT = 480
        self.tick_frequency = cv2.getTickFrequency() / 1000.
        self.font = cv2.FONT_HERSHEY_PLAIN
        self.font_scale = 1.0
        
        # État du système
        self.report = {}
        self.result = {"m1": -1, "h1": 0, "m2": -1, "h2": 0}
        self.loop_counter = 0
        self.img_cam = np.zeros((self.HEIGHT, self.WIDTH, 3), np.uint8)
        self.img_canny = np.zeros((self.HEIGHT, self.WIDTH), np.uint8)
        
        self.start = cv2.getTickCount()

        # Paramètres OpenCV
        self.param_canny_min = 150
        self.param_canny_max = 200
        cv2.setUseOptimized(True)
        self.br = CvBridge()

        # Calibration camera
        self.beta_x = 0.11499
        self.beta_y = 35.68
        self.u0 = 312.0
        self.vh = 116.18

        # Initialisation matériel
        self.camera = self._set_camera()
        #self.perspective_width = self._init_perspective()

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

    def calibration(self):
        self.beta_x = 0.11499
        self.beta_y = 35.68
        self.u0 = 312.0
        self.vh = 116.18


    def web_callback(self, msg):
        if msg.data == "save_img":
            filename = "/home/montaulab/Pictures/cam_" + str(cv2.getTickCount()) + ".jpg" 
            cv2.imwrite(filename, self.img_cam)

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
            start = cv2.getTickCount()
            result = func(self,*args, **kwargs)
            end = cv2.getTickCount()
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

        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "image/jpeg, width=640, height=480, framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink drop=1 max-buffers=1 sync=false"
        )
        gstreamer = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=(int)640, height=(int)480, framerate=(fraction)60/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        )

        cam = cv2.VideoCapture(gstreamer, cv2.CAP_GSTREAMER)
        return cam

    def _init_perspective(self):
        time.sleep(2)
        ret, img = self.camera.read()
        perspective_list = []
        img_gray = self._prepare_image_internal(img)
        img_canny = cv2.Canny(img_gray, self.param_canny_min, self.param_canny_max)
        
        for h in range(self.HEIGHT):
            edges = img_canny[h:h+1, :].ravel()
            bord = np.where(edges == 255)[0]
            delta = np.ediff1d(bord)
            delta = delta[delta > 10]
            perspective_list.append(int(delta.min()) if len(delta) > 0 else self.WIDTH)
            print(h,perspective_list[-1])
            
        #b0, b1 = self._linear_regression(np.array(range(self.HEIGHT)), np.array(perspective_list))
        b1 = (109.-15.)/(480.-123.)
        b0 = 109. - b1 * 480. 
        print(b0,b1)
        res = np.array([b0 + b1 * h for h in range(self.HEIGHT)])
        #res = np.array(perspective_list)
        print(f"Init perspective OK: Min={res.min()}, Max={res.max()}")
        return res

    def _linear_regression(self, x, y):
        """
        n = np.size(x)
        mx, my = np.mean(x), np.mean(y)
        ss_xy = np.sum(y * x) - n * my * mx
        ss_xx = np.sum(x * x) - n * mx * mx
        b1 = ss_xy / ss_xx
        b0 = my - b1 * mx
        """
        b1 = np.cov(x, y)[0, 1] / np.var(x)
        b0 = np.mean(y) - b1 * np.mean(x)
        return b0, b1

    @chronometre
    def analyse_line_new(self, gray, canny, h):
        w = 0.05
        wd = w * 0.2 
        beta_x = 0.11499
        beta_y = 35.68
        u0 = 312.0
        vh = 116.18

        edges = canny[h:h+1, :].ravel()

        bord = np.where(edges == 255)[0]
        if bord.shape[0]<4: # we want to have at least 4 edges.
            return -1
        fil = np.ediff1d(bord,to_end=False) > 3
        bord = bord[fil] # keeps only the large difference
        delta = np.ediff1d(bord)
        delta_x = (delta -u0) / (vh-h) * beta_x

        target = np.where(abs(delta - w) < wd)[0]
        band_dist = np.ediff1d(target)

        if len(target) == 3 and band_dist[0] == 1 and band_dist[1] == 1:
            return (bord[target[1]] + bord[target[1]+1]) // 2
        elif len(target) == 2 and band_dist[0] == 1:
            c0, c1 = (bord[target[0]] + bord[target[0]+1]) // 2, (bord[target[1]] + bord[target[1]+1]) // 2
            g_vals = gray[h:h+1, :].ravel()
            return c1 if g_vals[c0] < g_vals[c1] else c0
        return -1


    @chronometre
    def analyse_line(self, gray, canny, h):
        w = self.perspective_width[h]
        if w < 20: return -1
        wd = w * 0.2
        edges = canny[h:h+1, :].ravel()

        bord = np.where(edges == 255)[0]
        delta = np.ediff1d(bord)
        target = np.where(abs(delta - w) < wd)[0]
        band_dist = np.ediff1d(target)
        
        if len(target) == 3 and band_dist[0] == 1 and band_dist[1] == 1:
            return (bord[target[1]] + bord[target[1]+1]) // 2
        elif len(target) == 2 and band_dist[0] == 1:
            c0, c1 = (bord[target[0]] + bord[target[0]+1]) // 2, (bord[target[1]] + bord[target[1]+1]) // 2
            g_vals = gray[h:h+1, :].ravel()
            return c1 if g_vals[c0] < g_vals[c1] else c0
        return -1

    @chronometre
    def find_band(self, gray, canny, way):
        h = self.HEIGHT // 8 if way == 1 else self.HEIGHT * 7 // 8
        r_name = "B_find+" if way == 1 else "B_find-"
        k = 0
        while 0 <= h < self.HEIGHT:
            k += 1
            m = self.analyse_line_new(gray, canny, h)
            if m != -1:
                self.report[r_name] = k
                return m, h
            h += 10 * way
        self.report[r_name] = k
        return -1, 0

    @chronometre
    def read_camera(self):
        return self.camera.read()

    # --- Boucle Principale (Thread) ---
    def infinite_loop(self):
        while rclpy.ok():
            self.loop_item()

    @chronometre
    def _prepare_image_internal(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        eq = cv2.equalizeHist(gray)
        return cv2.GaussianBlur(eq, (5, 5), 0)
        #return eq 

    @chronometre
    def loop_item(self):

        frame = self.queue.get()
        prepare_start = cv2.getTickCount() 
        gray = self._prepare_image_internal(frame)
        canny = cv2.Canny(gray, self.param_canny_min, self.param_canny_max)
        prepare_end = cv2.getTickCount() 
        #print("delta prepare t",(prepare_end-prepare_start) / self.tick_frequency)

        self.result["m1"], self.result["h1"] = self.find_band(gray, canny, 1)
        self.result["m2"], self.result["h2"] = self.find_band(gray, canny, -1)

        self.result["x1"] = self.beta_x * ( self.result["m1"] - self.u0 ) / ( self.result["h1"] - self.vh )
        self.result["y1"] = self.beta_y / ( self.result["h1"] - self.vh )
        self.result["x2"] = self.beta_x * ( self.result["m2"] - self.u0 ) / ( self.result["h2"] - self.vh )
        self.result["y2"] = self.beta_y / ( self.result["h2"] - self.vh )
        self.publish_img_cam()
        self.publish_text_report()
        end = cv2.getTickCount()
        t = (end - self.start) / self.tick_frequency
        #print("delta t",t)
        self.start = end
        self.queue.task_done()

    @chronometre
    def loop_item_vpi(self):
        frame = self.queue.get()
        prepare_start = cv2.getTickCount() 
        input = vpi.asimage(np.asarray(frame))
        with vpi.Backend.CUDA:
             gray = input.convert(vpi.Format.U8).eqhist().box_filter(5, border=vpi.Border.ZERO)
             output = gray.canny(thresh_strong=self.param_canny_max, thresh_weak=self.param_canny_min, edge_value=255, nonedge_value=0, norm=vpi.Norm.L2)

        canny = output.cpu()
        prepare_end = cv2.getTickCount() 
        #print("delta prepare t",(prepare_end-prepare_start) / self.tick_frequency)
        self.img_canny = canny.copy()
        self.result["m1"], self.result["h1"] = self.find_band(gray, canny, 1)
        self.result["m2"], self.result["h2"] = self.find_band(gray, canny, -1)

        self.result["x1"] = self.beta_x * ( self.result["m1"] - self.u0 ) / ( self.result["h1"] - self.vh )
        self.result["y1"] = self.beta_y / ( self.result["h1"] - self.vh )
        self.result["x2"] = self.beta_x * ( self.result["m2"] - self.u0 ) / ( self.result["h2"] - self.vh )
        self.result["y2"] = self.beta_y / ( self.result["h2"] - self.vh )

        self.publish_img_cam()
        self.publish_text_report()
        end = cv2.getTickCount()
        t = (end - self.start) / self.tick_frequency
        #print("delta t",t)
        self.start = end
        self.queue.task_done()

    def publish_img_cam(self):
        img_draw = self.img_cam.copy()
        result = self.result.copy()
        m1 = self.result["m1"]
        m2 = self.result["m2"]
        h1 = self.result["h1"]
        h2 = self.result["h2"]

        if m1 != -1:
            cv2.circle(img_draw, (m1, h1), 5, (255, 0, 0), -1)
        if m2 != -1:
            cv2.circle(img_draw, (m2,h2), 5, (0, 255, 0), -1)
        #cv2.line(img_draw,(0,self.HEIGHT // 8),(self.WIDTH,self.HEIGHT // 8), (155,0,0),1)
        #cv2.line(img_draw,(0,self.HEIGHT * 7 // 8),(self.WIDTH,self.HEIGHT * 7 // 8), (0,155,0),1)
        #cv2.line(img_draw,(int(self.WIDTH//2-self.perspective_width[0]//2),0),(int(self.WIDTH//2-self.perspective_width[self.HEIGHT-1]//2),self.HEIGHT), (0,0,155),1)
        #cv2.line(img_draw,(int(self.WIDTH//2+self.perspective_width[0]//2),0),(int(self.WIDTH//2+self.perspective_width[self.HEIGHT-1]//2),self.HEIGHT), (0,0,155),1)
        self.publisher_img_cam.publish(self.br.cv2_to_imgmsg(img_draw))
        #self.publisher_img_cam.publish(self.br.cv2_to_imgmsg(self.img_canny))

    def publish_text_report(self):
        msg = String()
        x= {} 
        for k, v in self.result.items():
            if any(x in k for x in ["sum", "count"]): continue
            #msg.data += f"{k:>15}: {v:12.6f}\r\n" if isinstance(v, float) else f"{k:>15}: {v:>12}\r\n"
            x[k]=str(v)
        msg.data=json.dumps(x)
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

