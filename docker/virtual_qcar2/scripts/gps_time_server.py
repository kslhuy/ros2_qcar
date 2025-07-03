import socket
import time
import pickle

PORT = 8001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', PORT))
sock.settimeout(1.0)  # avoid hanging forever
print(f"GPS Time Server started on port {PORT}...")

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            if data == b"time_request":
                gps_time = time.time() + 2.0
                sock.sendto(pickle.dumps(gps_time), addr)
        except socket.timeout:
            continue
except KeyboardInterrupt:
    print("\n[GPS Server] Stopped by user.")
finally:
    sock.close()
