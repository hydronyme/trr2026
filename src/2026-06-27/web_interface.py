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
        self.subscriptionBrain = self.create_subscription(String,'brain_status',self.brain_callback, 10)
        self.subscriptionBrain
        self.publisher_command = self.create_publisher(String, 'web_command', 10)

        self.br = CvBridge()
        self.tick_frequency = cv2.getTickFrequency() / 1000.
        self.last_tick = 0
        self.tick = 0
        self.delta_time = ""
        self.buffer = 0
        self.msg = ""
        self.brain_status = ""
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
        
    def brain_callback(self, msg):
        self.brain_status = msg.data
        
    def gen_video_camera(self):
        return 0
        while True:
           if isinstance(self.buffer, np.ndarray):
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
        img = np.round(img_cumul).astype(np.uint8)

        height=img.shape[0]
        width=img.shape[1]
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

        @self.app.route('/brain')
        def v_brain(): 
            return self.brain_status

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
        self.app.run(host='0.0.0.0', port=5001, threaded=True)

def main(args=None):
    system = FlaskSubscriber()
    system.run()
