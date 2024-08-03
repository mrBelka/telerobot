import socket
import pyaudio
import pickle
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AudioClient(Node):
    def __init__(self, ip, port):
        super().__init__('audio_client_1')

        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100

        self.client_socket = None
        self.stream_out = None
        self.ip = ip
        self.port = port

        self.create_connection()
        self.subscription = self.create_subscription(
            String,
            'audio_connection_1',
            self.listener_callback,
            10)
        self.subscription

        self.create_timer(600, self.reconnect)

    def create_connection(self):
        try:
            # Socket create
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.ip, self.port))
            data = b""
            payload_size = struct.calcsize("Q")
            p = pyaudio.PyAudio()
            self.stream_out = p.open(format=self.FORMAT,
                                      channels=self.CHANNELS,
                                      rate=self.RATE,
                                      output=True)

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

                    frame_data = data[:msg_size]
                    data = data[msg_size:]
                    frame = pickle.loads(frame_data)
                    if frame is not None:
                        self.stream_out.write(frame)

        except Exception as e:
            self.get_logger().error(f"Server connection error: {e}")

        finally:
            if self.stream_out:
                self.stream_out.stop_stream()
                self.stream_out.close()
            if self.client_socket:
                self.client_socket.close()

    def reconnect(self):
        self.get_logger().info("Attempting to reconnect...")
        if self.stream_out:
            self.stream_out.stop_stream()
            self.stream_out.close()
        if self.client_socket:
            self.client_socket.close()

        try:
            self.create_connection()
            self.get_logger().info("Reconnected successfully.")
        except Exception as e:
            self.get_logger().error(f"Reconnection failed: {e}")

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    ip = '192.168.1.139'
    port = 1234
    audio_client = AudioClient(ip, port)
    rclpy.spin(audio_client)
    audio_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()