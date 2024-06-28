import socket, cv2, pickle, struct

def main(args=None):
    # Socket Create
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    host_ip =  '192.168.1.139'
    print('HOST IP:',host_ip)
    port = 9999
    socket_address = (host_ip,port)

    try:
        # Socket Bind
        server_socket.bind(socket_address)

        # Socket Listen
        server_socket.listen(5)
        print("LISTENING AT:",socket_address)

        # Socket Accept
        while True:
            client_socket,addr = server_socket.accept()
            print('GOT CONNECTION FROM:',addr)
            if client_socket:
                vid = cv2.VideoCapture(0)
                
                try:
                    while(vid.isOpened()):
                        img,frame = vid.read()
                        a = pickle.dumps(frame)
                        message = struct.pack("Q",len(a))+a
                        client_socket.sendall(message)
                        
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            client_socket.close()
                            break

                except Exception as e:
                    print(f"Data sending error: {e}")

                finally:
                    vid.release()
                    cv2.destroyAllWindows()
                    client_socket.close()

    except Exception as e:
        print(f"Server start error: {e}")

if __name__ == '__main__':
    main()