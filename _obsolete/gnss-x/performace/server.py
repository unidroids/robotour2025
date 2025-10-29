import socket

HOST = "127.0.0.1"
PORT = 5001

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen()
    print(f"📡 Server nasloucha na {HOST}:{PORT}")
    while True:
        conn, addr = s.accept()
        with conn:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                # echo back (jen pro test, jinak můžeš vynechat)
                conn.sendall(data)
