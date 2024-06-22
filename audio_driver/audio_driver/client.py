import socket, pyaudio, pickle, struct

def main(args=None):
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 44100

    # Create socket
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    host_ip = '192.168.1.139'
    port = 1234
    client_socket.connect((host_ip,port))
    data = b""
    payload_size = struct.calcsize("Q")
    p = pyaudio.PyAudio()
    stream_out = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    output=True)

    while True:
        while len(data) < payload_size:
            packet = client_socket.recv(4*1024)  # 4K
            if not packet:
                break
            data += packet
            
        if len(data) >= payload_size:
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("Q", packed_msg_size)[0]
            
            while len(data) < msg_size:
                data += client_socket.recv(4*1024)
                
            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data)
            if frame:
                stream_out.write(frame)

    stream_out.stop_stream()
    stream_out.close()
    client_socket.close()

if __name__ == '__main__':
    main()