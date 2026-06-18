#!/usr/bin/env python3
from collections import deque
import cv2 as cv
import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector(Node):

    def __init__(self):
        super().__init__('line_detector')

        self.publisher_       = self.create_publisher(Int32, '/line_error',       10)
        self.finish_pub       = self.create_publisher(Bool,  '/finish_line',       10)
        self.img_pub          = self.create_publisher(Image, '/camera/image',      10)
        #self.raw_pub          = self.create_publisher(Image, '/camera/raw',        10)
        self.camera_sub       = self.create_subscription(Image, '/r1/logi_camera/image', self.camera_callback, 10)

        self.bridge = CvBridge()

        self.get_logger().info('Nodo Suscriptor de Imagen Iniciado.')

        #self.cap = cv.VideoCapture('/dev/video0', cv.CAP_V4L2)
        #self.cap.set(cv.CAP_PROP_FRAME_WIDTH,  320)
        #self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
        #self.cap.set(cv.CAP_PROP_FPS,          30)
        #self.cap.set(cv.CAP_PROP_BUFFERSIZE,   1)

        #if not self.cap.isOpened():
            #self.get_logger().error("Cannot open camera /dev/video0")
            #exit()

        self.H_history      = deque(maxlen=10)
        self.error_history  = deque(maxlen=5)
        self.last_error     = 0

        # Anti-spam para intersección
        

        #self.timer = self.create_timer(0.033, self.process_frame)
        #self.get_logger().info("LineDetector Hough iniciado")
        


    def camera_callback(self, msg):
            # 3. Convertimos el mensaje de ROS 2 a una imagen de OpenCV (BGR)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 4. Mostramos la imagen en una ventana de OpenCV

            import time
            now = time.time()

            h, w = frame.shape[:2]

            #self.raw_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

            # ---- Finish line ----
            finish = self.detect_finish_line(frame)
            fm = Bool(); fm.data = finish
            self.finish_pub.publish(fm)


            # ---- ROI muy baja y estrecha — solo sigue línea del centro ----
            roi   = frame[int(h * 0.72):h, :]
            roi_h, roi_w = roi.shape[:2]
            debug = roi.copy()

            H_roi    = roi[int(roi_h * 0.5):, int(roi_w * 0.3):int(roi_w * 0.7)]
            hsv      = cv.cvtColor(H_roi, cv.COLOR_BGR2HSV)
            H_mean   = np.mean(hsv[:, :, 2])
            self.H_history.append(H_mean)
            H_smooth = np.mean(self.H_history)
            #cutting  = self.get_threshold(H_smooth)
            cutting = 80


            gray    = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
            blurred = cv.GaussianBlur(gray, (5, 5), 0)
            _, binary = cv.threshold(blurred, cutting, 255, cv.THRESH_BINARY_INV)
            cv.imshow("Camara ROS 2", binary)
            cv.waitKey(1)

            # Trapecio estrecho centrado — ignora líneas de los costados
            cx_roi   = roi_w // 2
            top_half = int(roi_w * 0.12)   # ancho arriba
            bot_half = int(roi_w * 0.4)   # ancho abajo 0.028 oooo 0.5
            top_y    = int(roi_h * 0.05)
            trap = np.array([[
                (cx_roi - top_half, top_y),
                (cx_roi + top_half, top_y),
                (cx_roi + bot_half, roi_h),
                (cx_roi - bot_half, roi_h)
            ]], dtype=np.int32)

            mask   = np.zeros((roi_h, roi_w), dtype=np.uint8)
            cv.fillPoly(mask, trap, 255)
            masked = cv.bitwise_and(binary, binary, mask=mask)

            k     = np.ones((3, 3), np.uint8)
            morph = cv.erode(masked, k, iterations=1)
            morph = cv.dilate(morph, k, iterations=2)

            edges = cv.Canny(morph, 50, 150)
            lines = cv.HoughLinesP(edges, 1, np.pi / 180,
                                threshold=18,
                                minLineLength=12,
                                maxLineGap=30)

            ref_x   = roi_w // 2
            error_x = self.last_error

            cv.polylines(debug, trap, True, (255, 0, 255), 1)
            cv.line(debug, (ref_x, 0), (ref_x, roi_h), (255, 0, 0), 1)

            if lines is not None:
                valid = []
                for l in lines:
                    x1, y1, x2, y2 = l[0]
                    angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
                    if 20 < angle < 160:
                        valid.append((x1, y1, x2, y2))

                if valid:
                    total_len = 0.0
                    wx, wy    = 0.0, 0.0
                    for x1, y1, x2, y2 in valid:
                        seg = np.hypot(x2 - x1, y2 - y1)
                        wx += (x1 + x2) / 2.0 * seg
                        wy += (y1 + y2) / 2.0 * seg
                        total_len += seg
                        cv.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 1)

                    cx = wx / total_len
                    cy = wy / total_len

                    error_x = int(cx - ref_x)
                    self.error_history.append(error_x)
                    error_x = int(np.mean(self.error_history))
                    self.last_error = error_x

                    cv.circle(debug, (int(cx), int(cy)), 5, (0, 255, 255), -1)
                    cv.line(debug, (ref_x, roi_h), (int(cx), int(cy)), (0, 255, 255), 1)
            else:
                cv.putText(debug, "Sin linea", (5, roi_h - 5),
                        cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            msg = Int32(); msg.data = error_x
            self.publisher_.publish(msg)

            color = (0, 0, 255) if finish else (0, 255, 255)
            cv.putText(debug, f"E:{error_x}", (3, 15),
                    cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv.putText(debug, f"B:{H_smooth:.0f} C:{cutting}", (3, 30),
                    cv.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
            if finish:
                cv.putText(debug, "META!", (ref_x - 20, roi_h // 2),
                        cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            self.get_logger().info(
                f'E:{error_x} B:{H_smooth:.0f} C:{cutting} '
                f'Finish:{finish}')

            self.img_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def get_threshold(self, H):
        if   H < 110: return 100
        elif H < 135: return 140
        elif H < 150: return 140
        elif H < 155: return 115
        elif H < 165: return 120
        elif H < 170: return 125
        elif H < 190: return 130
        elif H < 200: return 140
        else:         return 160

    def detect_finish_line(self, frame):
        h, w = frame.shape[:2]
        roi  = frame[int(h * 0.60):int(h * 0.80), int(w * 0.20):int(w * 0.80)]
        hsv  = cv.cvtColor(roi, cv.COLOR_BGR2HSV)

        y_mask = cv.inRange(hsv, np.array([18, 80, 80]),  np.array([35, 255, 255]))
        b_mask = cv.inRange(hsv, np.array([0,  0,  0]),   np.array([180,255, 50]))

        total  = roi.shape[0] * roi.shape[1]
        yr     = cv.countNonZero(y_mask) / total
        br     = cv.countNonZero(b_mask) / total

        return yr > 0.12 and br > 0.10 and (yr + br) > 0.30

        

    def destroy_node(self):
        #if self.cap.isOpened():
            #self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()