import socket, cv2, pickle, struct, rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CamServer(Node):
    def __init__(self, ip, port):
        super().__init__('cam_server')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host_ip = ip
        self.port = port

        try:
            self.server_socket.bind((self.host_ip, self.port))
            self.server_socket.listen(5)
            self.get_logger().info(f"Listening at: {self.host_ip}:{self.port}")
            self.publisher_ = self.create_publisher(String, 'cam_connection', 10)

            while True:
                client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f"Got connection from: {addr}")

                if client_socket:
                    vid = cv2.VideoCapture(0)
                    try:
                        while vid.isOpened():
                            img, frame = vid.read()
                            a = pickle.dumps(frame)
                            message = struct.pack("Q", len(a)) + a
                            client_socket.sendall(message)
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                client_socket.close()
                                break

                    except Exception as e:
                        self.get_logger().error(f"Data sending error: {e}")

                    finally:
                        vid.release()
                        cv2.destroyAllWindows()
                        client_socket.close()

        except Exception as e:
            self.get_logger().error(f"Server start error: {e}")

def main(args=None):
    rclpy.init(args=args)
    ip = '192.168.1.89'
    port = 9999
    cam_server = CamServer(ip, port)
    rclpy.spin(cam_server)
    cam_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()