import socket, pyaudio, pickle, struct

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100

# Socket create
server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_name = socket.gethostname()
host_ip = '192.168.1.43'
print('HOST IP:', host_ip)
port = 1234
socket_address = (host_ip,port)

# Socket bind
server_socket.bind(socket_address)

# Socket listen
server_socket.listen(5)
print("LISTENING AT:", socket_address)

# Socket accept
while True:
    client_socket,addr = server_socket.accept()
    print('GOT CONNECTION FROM:',addr)
    if client_socket:
        p = pyaudio.PyAudio()
        
        stream_in = p.open(format=FORMAT,
                   channels=CHANNELS,
                   rate=RATE,
                   input=True,
                   frames_per_buffer=CHUNK)
        
        while True:
            data = stream_in.read(CHUNK)
            a = pickle.dumps(data)
            message = struct.pack("Q", len(a))+a
            client_socket.sendall(message)