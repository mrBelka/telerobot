import socket, pyaudio, pickle, struct, rclpy
from rclpy.node import Node

class AudioServer(Node):
    def __init__(self, ip, port):
        super().__init__('audio_server')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host_ip = ip
        self.port = port

        try:
            self.server_socket.bind((self.host_ip, self.port))
            self.server_socket.listen(5)
            self.get_logger().info(f"Listening at: {self.host_ip}:{self.port}")

            while True:
                client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f"Got connection from: {addr}")

                if client_socket:
                    p = pyaudio.PyAudio()
                    stream_in = p.open(format=pyaudio.paInt16,
                                       channels=1,
                                       rate=44100,
                                       input=True,
                                       frames_per_buffer=1024)

                    try:
                        while True:
                            data = client_socket.recv(1024)
                            if not data:
                                break
                            a = pickle.loads(data)
                            msg = struct.pack("Q", len(a)) + a
                            stream_in.write(msg)

                    except Exception as e:
                        self.get_logger().error(f"Data receiving error: {e}")

                    finally:
                        stream_in.stop_stream()
                        stream_in.close()
                        client_socket.close()
                        p.terminate()

        except Exception as e:
            self.get_logger().error(f"Server start error: {e}")

        finally:
            self.server_socket.close()

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