import socket, cv2, pickle, struct, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self, ip, port):
        super().__init__('video_publisher')
        self.ip = ip
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host_ip = ip
        self.port = port
        self.socket_address = (self.host_ip, self.port)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'video_stream', 10)

    def start_server(self):
        try:
            self.server_socket.bind(self.socket_address)
            self.server_socket.listen(5)
            self.get_logger().info(f"Listening at: {self.socket_address}")

            while True:
                client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f"Got connection from: {addr}")

                if client_socket:
                    vid = cv2.VideoCapture(0)

                    try:
                        while vid.isOpened():
                            img, frame = vid.read()
                            a = pickle.dumps(frame)
                            msg = struct.pack("Q", len(a)) + a
                            client_socket.sendall(msg)
                            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                            self.publisher_.publish(img_msg)

                    except Exception as e:
                        self.get_logger().error(f"Data sending error: {e}")

                    finally:
                        client_socket.close()
                        vid.release()
                        cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error(f"Server start error: {e}")

def main(args=None):
    rclpy.init(args=args)
    ip = '192.168.1.62'
    port = 9999
    video_publisher = VideoPublisher(ip, port)
    video_publisher.start_server()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()