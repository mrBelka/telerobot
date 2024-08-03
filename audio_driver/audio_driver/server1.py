import socket, pyaudio, pickle, struct, rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AudioServer(Node):
    def __init__(self, ip, port):
        super().__init__('audio_server_1')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host_ip = ip
        self.port = port
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100
        msg = String()

        try:
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host_ip, self.port))
            self.server_socket.listen(5)
            self.get_logger().info(f"Listening at: {self.host_ip}:{self.port}")
            self.publisher_ = self.create_publisher(String, 'audio_connection_1', 10)

            while True:
                msg.data = "Not connected"
                self.publisher_.publish(msg)
                client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f"Got connection from: {addr}")
                if client_socket:
                    p = pyaudio.PyAudio()
                    stream_in = p.open(format=self.FORMAT,
                                       channels=self.CHANNELS,
                                       rate=self.RATE,
                                       input=True,
                                       frames_per_buffer=self.CHUNK)

                    try:
                        while True:
                            msg.data = f"Connected: {self.host_ip}:{self.port}"
                            self.publisher_.publish(msg)
                            data = stream_in.read(self.CHUNK)
                            a = pickle.dumps(data)
                            message = struct.pack("Q", len(a)) + a
                            client_socket.sendall(message)

                    except Exception as e:
                        self.get_logger().error(f"Data sending error: {e}")

                    finally:
                        stream_in.stop_stream()
                        stream_in.close()
                        client_socket.close()
                        p.terminate()

        except Exception as e:
            self.get_logger().error(f"Server start error: {e}")

def main(args=None):
    rclpy.init(args=args)
    ip = '192.168.1.139'
    port = 1234
    audio_server = AudioServer(ip, port)
    rclpy.spin(audio_server)
    audio_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()