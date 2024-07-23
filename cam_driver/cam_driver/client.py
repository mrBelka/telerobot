import socket
import cv2
import pickle
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CamClient(Node):
    def __init__(self, ip, port):
        super().__init__('cam_client')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host_ip = ip
        self.port = port
        self.bridge = CvBridge()

        try:
            self.client_socket.connect((self.host_ip, self.port))
            self.get_logger().info(f"Connected to: {self.host_ip}:{self.port}")

            self.subscription = self.create_subscription(
                Image,
                'image',
                self.listener_callback,
                10
            )
            self.subscription

            data = b""
            payload_size = struct.calcsize("Q")
            while True:
                while len(data) < payload_size:
                    packet = self.client_socket.recv(4 * 1024)
                    if not packet:
                        break
                    data += packet

                if len(data) >= payload_size:
                    packed_msg_size = data[:payload_size]
                    data = data[payload_size:]
                    msg_size = struct.unpack("Q", packed_msg_size)[0]
                    while len(data) < msg_size:
                        data += self.client_socket.recv(4 * 1024)
                    if len(data) >= msg_size:
                        frame_data = data[:msg_size]
                        data = data[msg_size:]
                        frame = pickle.loads(frame_data)
                        if frame is not None:
                            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

                            cv2.imshow("Receiving video", frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                break

        except Exception as e:
            self.get_logger().error(f"Server connection error: {e}")
        finally:
            self.client_socket.close()
            cv2.destroyAllWindows()

    def listener_callback(self, msg):
        self.get_logger().info("Received image")

def main(args=None):
    rclpy.init(args=args)
    ip = '192.168.1.139'
    port = 9999
    cam_client = CamClient(ip, port)
    rclpy.spin(cam_client)
    cam_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

