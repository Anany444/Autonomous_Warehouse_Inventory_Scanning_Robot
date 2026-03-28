import cv2
import json
from ultralytics import YOLO
from zxingcpp import read_barcode
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory
import os
         
pkg_path = get_package_share_directory('warehouse_mission_control')
model_path = os.path.join(pkg_path, 'models', 'qr_model.pt')

class QRScanner(Node):
    def __init__(self,
                 cam_index=0,
                 resolution=(640, 480),
                 exposure=-5,
                 pad=12):
        super().__init__("qr_pipeline")
        
        self.sub_cam = self.create_subscription(Image, "/camera/image_raw", self.camera_callback, 1)
        self.pub_cam = self.create_publisher(Image, "/qr_model/output_image", 1)
        self.pub_qr_string = self.create_publisher(String, "/qr_model/output_string", 1)
        
        self.start_scan_service = self.create_service(Trigger, "start_scan", self.start_scan_callback)
        self.stop_scan_service = self.create_service(Trigger, "stop_scan", self.stop_scan_callback)

        # Config
        self.model = YOLO(model_path)
        self.cam_index = cam_index
        self.resolution = resolution
        self.exposure = exposure
        self.pad = pad

        # State
        self.cap = None
        self.unique_qrs = set()
        self.qr_model_active = False
        
        self.bridge = CvBridge()
        self.output_frame = None
        self.current_decoded_qr = ""
        
    def start_scan_callback(self, request, response):
        self.get_logger().info("Starting QR scan...")
        self.qr_model_active = True
        response.success = True
        response.message = "QR scan started."
        return response
        
    def stop_scan_callback(self, request, response):
        self.get_logger().info("Stopping QR scan...")
        self.qr_model_active = False
        response.success = True
        response.message = "QR scan stopped."
        return response

    def camera_callback(self, msg):
        if not self.qr_model_active:
            return
        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_frame(frame)
        result_img = Image()  # Image with bounding boxes drawn
        result_img_msg = self.bridge.cv2_to_imgmsg(self.output_frame, encoding="bgr8")
        self.pub_cam.publish(result_img_msg)

    # ---------- Preprocessing ----------
    def preprocess(self, crop):
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 3)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)

        bw = cv2.adaptiveThreshold(
            enhanced, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            11, 2
        )

        return [enhanced, bw]

    # ---------- Decode ----------
    def decode(self, crop):
        for img in self.preprocess(crop):
            result = read_barcode(img)
            if result and result.text:
                return result.text
        return None

    # ---------- Processing ----------
    def process_frame(self, frame):
        results = self.model(frame, verbose=False)

        if not results[0].boxes:
            self.output_frame = frame
            return

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # padding
            x1 = max(0, x1 - self.pad)
            y1 = max(0, y1 - self.pad)
            x2 = min(frame.shape[1], x2 + self.pad)
            y2 = min(frame.shape[0], y2 + self.pad)

            crop = frame[y1:y2, x1:x2]

            if crop.shape[0] < 40 or crop.shape[1] < 40:
                continue

            qr_data = self.decode(crop)
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            if qr_data and (qr_data not in self.unique_qrs):
                print("NEW QR:", qr_data)
                self.current_decoded_qr = qr_data
                self.unique_qrs.add(qr_data)
                self.pub_qr_string.publish(String(data=qr_data))
                
                # Draw text
                cv2.putText(frame, qr_data, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
                self.output_frame = frame
                #cv2.imwrite(f"{qr_data}.jpg", crop)
                return
                try:
                    with open("qr_log.jsonl", "a") as f:
                        json.dump({"qr": qr_data}, f)
                        f.write("\n")
                except Exception as e:
                    print("JSON write error:", e)
        self.output_frame = frame

# ---------- Main ----------
def main(args=None):
    rclpy.init(args=args)
    scanner = QRScanner()
    try:
        rclpy.spin(scanner)
    except KeyboardInterrupt:
        for qr in scanner.unique_qrs:
            scanner.pub_cam.publish(String(data=qr))
        pass
    finally:
        scanner.destroy_node()
        rclpy.shutdown()
    


if __name__ == "__main__":
    main()