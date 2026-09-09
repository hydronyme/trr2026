import threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
from flask import Flask, Response, request
from flask import jsonify,render_template
import time
import os
import numpy as np
import math


from ament_index_python.packages import get_package_share_directory



class FlaskSubscriber(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('web_module')
        self.subscriptionImage = self.create_subscription(Image, 'video_frames', self.img_callback, 1)
        self.subscriptionImage
        self.subscriptionTopic = self.create_subscription(String,'camera',self.topic_callback, 10)
        self.subscriptionTopic
        self.publisher_command = self.create_publisher(String, 'web_command', 10)

        self.br = CvBridge()
        self.tick_frequency = cv2.getTickFrequency() / 1000.
        self.last_tick = 0
        self.tick = 0
        self.delta_time = ""
        self.buffer = 0
        self.msg = ""
        self.current_frame = 0
        self.package_path = get_package_share_directory('course_v1')

        self.app = Flask(
            __name__,
            template_folder=os.path.join(self.package_path, 'templates'),
            static_folder=os.path.join(self.package_path, 'static')
        ) 
        self._setup_routes()

    def img_callback(self, data):
        self.tick = cv2.getTickCount()
        self.delta_time = f"{(self.tick - self.last_tick) / self.tick_frequency}"
        self.last_tick = self.tick
        
        #self.get_logger().info('Receiving video frame:'+self.delta_time)
        self.current_frame = self.br.imgmsg_to_cv2(data)
        _, self.buffer = cv2.imencode('.jpg', self.current_frame)

    def topic_callback(self, msg):
        self.msg = msg.data
        
    def gen_video_camera(self):
        while True:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + self.buffer.tobytes() + b'\r\n')
            time.sleep(0.05)

    def gen_video_calibration(self):


        img_cumul =  np.zeros_like(self.current_frame)
        img_cumul = img_cumul.astype(float)

        k=0
        while k<10:
           cv2.accumulateWeighted(self.current_frame,img_cumul,0.1)
           k+=1
           time.sleep(.2)

        #img = self.current_frame
        img = np.round(img_cumul).astype(np.uint8)
        height=img.shape[0]
        width=img.shape[1]
        img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        vec_channels = cv2.split(img_yuv);
        vec_channels = list(vec_channels)
        vec_channels[0] = cv2.equalizeHist(vec_channels[0])
        img_yuv = cv2.merge(vec_channels)
        img_green = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
        mg = img_green.copy()
        mg[:,:,0]=0
        mg[:,:,2]=0
        img_gray = cv2.cvtColor(mg,cv2.COLOR_BGR2GRAY)

        #hsv = cv2.cvtColor(img_green, cv2.COLOR_BGR2HSV)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #img_gray = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        #img_gray = cv2.cvtColor(img_gray, cv2.COLOR_BGR2GRAY)
        ## Slice the green
        #imask = mask > 0
        #green = np.zeros_like(img_gray)
        #green[imask] = img_gray[imask]

        # find contours in the binary image
        ret, thresh = cv2.threshold(img_gray, 50, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print("-------------")
        k=0
        r= {} 
        delta = 0.03

        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        for c in contours:
           M = cv2.moments(c)

           if M["m00"] == 0.:
               continue
           u = int(M["m10"] / M["m00"])
           v = int(M["m01"] / M["m00"])
           cv2.circle(img, (u, v), 5, (255, 255, 255), 1)
           x = (k % 3)*delta
           y = 0.15 + math.floor(k/3)*delta
           imgpoints.append((u,v))
           objpoints.append((x,y,0))
           print(u,v,k % 3,math.floor(k/3),x,y)
           r[str(k%3)+"-"+str(math.floor(k/3))]={"x":x,"y":y,"u":u,"v":v}
           cv2.putText(img, str(k%3)+","+str(math.floor(k/3)), (u - 150, v+20 ),cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
           k+=1

        try:
            u0 = r["0-0"]["u"]
            print("u0",u0)
            vh = (r["0-0"]["y"]*r["0-0"]["v"]-r["0-3"]["y"]*r["0-3"]["v"]) / (r["0-0"]["y"]-r["0-3"]["y"])
            print("vh",vh)
            beta_v = r["0-0"]["y"] * (r["0-0"]["v"] - vh)
            print("beta_v",beta_v)
            beta_u = r["2-0"]["x"] * (r["2-0"]["v"] - vh) / (r["2-0"]["u"] - u0)
            print("beta_u",beta_u)

            for k in r.keys():
                x= beta_u * (r[k]["u"] - u0) / ( r[k]["v"] - vh) 
                y= beta_v / ( r[k]["v"] - vh) 
                print(k,r[k]["x"],x)
                print(k,r[k]["y"],y)

            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_gray.shape[::-1], None, None)
            print(mtx)
        except:
            pass

        """
        lower_black = np.array([0,0,0])  # BGR-code of your lowest red
        upper_black = np.array([30,30,30])   # BGR-code of your highest red
        img_gray = cv2.cvtColor(img[200:,:,:], cv2.COLOR_BGR2GRAY)
        img_eq = cv2.equalizeHist(img_gray)
        max_value = 255
        max_binary_value = 255
        trackbar_type = 'Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted'
        threshold_value = 100

        _, img_dst = cv2.threshold(img_eq, threshold_value, max_binary_value, 2 )
        mask = cv2.inRange(img_dst, 0, 100)
        coords=cv2.findNonZero(mask)
        #for coord in coords:
        #    cv2.circle(img_gray,coord[0], 5, 0, 1)
        #print(coords)
        """

        cv2.line(img,(int(width/2),0),(int(width/2),height-1),(255,0,0),1)
        _, buffer_calibration = cv2.imencode('.jpg', img)
        return (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer_calibration.tobytes() + b'\r\n')
        #yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer_calibration.tobytes() + b'\r\n')
            #time.sleep(2)

    def _setup_routes(self):

        @self.app.route('/calibration')
        def v_calibration(): 
            return Response(self.gen_video_calibration(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/camera')
        def v_cam(): 
            return Response(self.gen_video_camera(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/report')
        def v_report(): 
            return self.msg

        @self.app.route('/')
        def v_main(): 
            return render_template('command.html')

        @self.app.route('/engine_on')
        @self.app.route('/engine_off')
        @self.app.route('/engine_vmax')
        @self.app.route('/mode_auto')
        @self.app.route('/mode_manu')
        @self.app.route('/save_img')
        @self.app.route('/manu_angle')
        @self.app.route('/manu_speed_forward')
        @self.app.route('/manu_speed_backward')
        def web_command(): 
            msg = String()
            if request.path == "/engine_vmax": 
                vmax = request.args.get("vmax")
                msg.data="vmax:"+vmax
            elif request.path == "/manu_speed_forward": 
                speed = request.args.get("speed")
                msg.data="manu_speed_forward:"+speed
            elif request.path == "/manu_speed_backward": 
                speed = request.args.get("speed")
                msg.data="manu_speed_backward:"+speed
            elif request.path == "/manu_angle": 
                angle = request.args.get("angle")
                msg.data="manu_angle:"+angle
            else:
                msg.data=request.path[1:]
            self.publisher_command.publish(msg)
            return jsonify({"status":msg.data})

    def run_ros(self):
        node = self	
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def run(self):
        threading.Thread(target=self.run_ros,daemon=True).start()
        self.app.run(host='0.0.0.0', port=5000, threaded=True)

def main(args=None):
    system = FlaskSubscriber()
    system.run()
