import socket, pyaudio, pickle, struct, rclpy
from rclpy.node import Node

class AudioClientNode(Node):
    def __init__(self, ip, port):
        super().__init__('audio_client_node')

        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100

        # Socket create
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host_ip = '192.168.1.139'
        port = 1234

        try:
            self.client_socket.connect((host_ip, port))
            data = b""
            payload_size = struct.calcsize("Q")
            p = pyaudio.PyAudio()
            stream_out = p.open(format=FORMAT,
                                channels=CHANNELS,
                                rate=RATE,
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
                        stream_out.write(frame)

        except Exception as e:
            self.get_logger().error(f"Server connection error: {e}")

        finally:
            stream_out.stop_stream()
            stream_out.close()
            self.client_socket.close()
            p.terminate()

def main(args=None):
    rclpy.init(args=args)
    ip = '192.168.1.139'
    port = 1234
    audio_client_node = AudioClientNode(ip, port)
    rclpy.spin(audio_client_node)
    audio_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()