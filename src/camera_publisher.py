import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class CameraPublisher(Node):

    def __init__(self):

        super().__init__("camera_publisher")

        self.declare_parameter("camera_id", 0)
        self.declare_parameter("fps", 60.0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)

        self._set_camera()
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "video_frame", 1)
        self.timer = self.create_timer(1.0 / fps, self.publish_image)
        self.get_logger().info("Camera publisher started : mais lequel ?")

      # --- Initialisation ---
    def _set_camera(self):

        camera_id = self.get_parameter("camera_id").value
        fps = self.get_parameter("fps").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        print("============ coucou ===============")

        """
        self.cap = cv.VideoCapture(camera_id)

        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv.CAP_PROP_FPS, fps)
        """

        self.get_logger().info("coucou")
        gstreamer = (
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)60/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        )

        self.cap = cv.VideoCapture(gstreamer, cv.CAP_GSTREAMER)
        self.get_logger().info(self.cap)

        if not self.cap.isOpened():
            raise RuntimeError("Impossible d'ouvrir la caméra")

        return 

    def publish_image(self):

        ret, frame = self.cap.read()
        print(self.get_clock().now())
        if not ret:
            self.get_logger().warning("Frame non disponible")
            print("Frame non disponible")
            return

        print("Frame ok")
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

