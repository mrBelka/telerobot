import socket,cv2, pickle,struct

def main(args=None):
  # Socket create
  client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
  host_ip = '192.168.1.139'
  port = 9999
  client_socket.connect((host_ip,port))
  data = b""
  payload_size = struct.calcsize("Q")
  while True:
      while len(data) < payload_size:
          packet = client_socket.recv(4*1024)
          if not packet:
              break
          data += packet

      if len(data) >= payload_size:
          packed_msg_size = data[:payload_size]
          data = data[payload_size:]
          msg_size = struct.unpack("Q", packed_msg_size)[0]

          while len(data) < msg_size:
              data += client_socket.recv(4*1024)

          if len(data) >= msg_size:
              frame_data = data[:msg_size]
              data = data[msg_size:]
              frame = pickle.loads(frame_data)
              if frame is not None:
                  cv2.imshow("RECEIVING VIDEO", frame)
                  key = cv2.waitKey(1) & 0xFF
                  if key == ord('q'):
                      break

  client_socket.close()
  
if __name__ == '__main__':
  main()