#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/r1/logi_camera/image',
            self.image_callback,
            10 
        )
        self.subscription  

        self.inter_pub = self.create_publisher(Bool, '/intersection_line', 10)
        
   
        self.bridge = CvBridge()
        self.get_logger().info('Nodo Suscriptor de Imagen Iniciado.')

    def image_callback(self, msg):
        try:
      
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camara ROS 2", cv_image)

            h, w = cv_image.shape[:2] 

            roi = cv_image[int(h*0.6):h, :]
            roi_h, roi_w = roi.shape[:2]
            gris_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gris_image, (5, 5), 0)
            _, binary = cv2.threshold(blurred,80, 255, cv2.THRESH_BINARY_INV)
            cv2.imshow("Binary", binary)

            #Morphological operations
            kernel = np.ones((3,3), np.uint8)

            morph = cv2.erode(binary, kernel, iterations=1)

            morph = cv2.dilate(morph, kernel, iterations=1)

            #cv2.imshow("morph", binary)

            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(morph, connectivity=8)

            zona_central_min = int(roi_h * 0.40)
            zona_central_max = int(roi_h * 0.60)

            contador_candidatos = 0

            for i in range(1, num_labels):
                x, y, bw, bh, area = stats[i]
                cx, cy = centroids[i]
                #print("Area objeto " + str(i) + ": " + str(area))

                if area >= 80:
                    if zona_central_min <= cy <= zona_central_max:
                        contador_candidatos += 1

            msg_interseccion = Bool()

            if contador_candidatos >= 4:
                print("Interseccion")
                msg_interseccion.data = True
                self.get_logger().info("¡Intersección Detectada! Enviando True...")
            else:
                msg_interseccion.data = False
            
            # Se publica el estado en cada frame para mantener el tópico activo
            self.inter_pub.publish(msg_interseccion)

                
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Cerrando ventana por petición del usuario.')
                
        except CvBridgeError as e:
            self.get_logger().error(f'Error al convertir la imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()