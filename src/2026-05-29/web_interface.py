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
        self.subscriptionTopic = self.create_subscription(String,'camera_analysis',self.topic_callback, 10)
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
        """

        v1=400
        v2=850
        u1=180
        #v1=300
        #v2=450
        #u1=180
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_gray[:,:v1]=255
        img_gray[:,v2:]=255
        img_gray[:u1,:]=255 # partie haute
        cv2.line(img,(v1,0),(v1,img.shape[0]),(255,0,0),1)
        cv2.line(img,(v2,0),(v2,img.shape[0]),(255,0,0),1)
        cv2.line(img,(0,u1),(img.shape[1],u1),(255,0,0),1)
        # find contours in the binary image
        ret, thresh = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        print("-------------")
        k=0
        r= {}
        y0= 0.162
        delta = 0.0265

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
            y = y0 + math.floor(k/3)*delta
            imgpoints.append((u,v))
            objpoints.append((x,y,0))
            print(u,v,k % 3,math.floor(k/3),x,y)
            r[str(k%3)+"-"+str(math.floor(k/3))]={"x":x,"y":y,"u":u,"v":v}
            cv2.putText(img, str(k%3)+","+str(math.floor(k/3)), (u - 150, v+20 ),cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
            k+=1

        try:
            u0 = r["0-0"]["u"]
            print("u0",u0)
            vh = (r["0-0"]["y"]*r["0-0"]["v"]-r["0-4"]["y"]*r["0-4"]["v"]) / (r["0-0"]["y"]-r["0-4"]["y"])
            print("vh",vh)
            beta_v = r["0-0"]["y"] * (r["0-0"]["v"] - vh)
            print("beta_v",beta_v)
            beta_u = r["2-0"]["x"] * (r["2-0"]["v"] - vh) / (r["2-0"]["u"] - u0)
            print("beta_u",beta_u)
            cv2.putText(img, "u0="+str(u0), (10,400),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(img, "vh="+str(vh), (10,420),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(img, "beta_u="+str(beta_u), (10,440),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv2.putText(img, "beta_v="+str(beta_v), (10,460),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

            for k in r.keys():
               x= beta_u * (r[k]["u"] - u0) / ( r[k]["v"] - vh)
               y= beta_v / ( r[k]["v"] - vh)
               print(k,r[k]["x"],x)
               print(k,r[k]["y"],y)

            #ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera([objpoints], [imgpoints], img_gray.shape[::-1], None, None)
            #print(mtx)

        except:
            pass
        """
        cv2.line(img,(int(width/2),0),(int(width/2),height-1),(255,0,0),1)
        _, buffer_calibration = cv2.imencode('.jpg', img)
        return (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer_calibration.tobytes() + b'\r\n')

    def _setup_routes(self):

        @self.app.route('/calibration')
        def v_calibration(): 
            return render_template('calibration.html')
            #return Response(self.gen_video_calibration(), mimetype='multipart/x-mixed-replace; boundary=frame')

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
