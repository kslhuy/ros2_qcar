# import socket
# import time
# import pickle

# PORT = 8001
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.bind(('0.0.0.0', PORT))
# sock.settimeout(1.0)  # avoid hanging forever
# print(f"GPS Time Server started on port {PORT}...")

# try:
#     while True:
#         try:
#             data, addr = sock.recvfrom(1024)
#             if data == b"time_request":
#                 gps_time = time.time() + 2.0
#                 sock.sendto(pickle.dumps(gps_time), addr)
#         except socket.timeout:
#             continue
# except KeyboardInterrupt:
#     print("\n[GPS Server] Stopped by user.")
# finally:
#     sock.close()

import socket
import time
import json as ujson  # Faster JSON library
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] GPS_SERVER: %(message)s',
    handlers=[
        logging.FileHandler('gps_server.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

PORT = 8001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', PORT))
sock.settimeout(1.0)  # Avoid hanging forever
logger.info(f"GPS Time Server started on port {PORT}...")

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            if data == b"time_request":
                gps_time = time.time() + 2.0
                sock.sendto(ujson.dumps(gps_time).encode(), addr)
                logger.info(f"Sent GPS time {gps_time:.3f} to {addr}")
        except socket.timeout:
            continue
        except ujson.JSONEncodeError as e:
            logger.error(f"JSON encode error: {e}")
        except Exception as e:
            logger.error(f"Error: {e}")
except KeyboardInterrupt:
    logger.info("Stopped by user.")
finally:
    sock.close()
