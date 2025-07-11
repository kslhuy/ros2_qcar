import socket
import json as ujson
import time
import random
import logging
from md_logging_config import logger as parent_logger

class GPSSync:
    """GPS-like time sync via centralized time server."""
    def __init__(self, gps_server_ip='127.0.0.1', gps_server_port=8001):
        self.gps_time_offset = 0
        self.last_sync_time = time.time()
        self.gps_server_ip = gps_server_ip
        self.gps_server_port = gps_server_port
        self.logger = logging.LoggerAdapter(parent_logger, {'vehicle_id': 'GPS'})
        self.last_valid_offset = 0  # Store last valid offset

    def request_gps_time(self):
        """Fetch GPS time from external server."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        try:
            sock.sendto(b"time_request", (self.gps_server_ip, self.gps_server_port))
            data, _ = sock.recvfrom(1024)
            gps_time = ujson.loads(data.decode())
            # Simulate GPS jitter (±10ms)
            gps_time += random.uniform(-0.01, 0.01)
            return gps_time
        except Exception as e:
            self.logger.error(f"Error contacting GPS server: {e}")
            return time.time() + self.last_valid_offset  # Use last valid offset
        finally:
            sock.close()

    def sync_with_gps(self):
        gps_time = self.request_gps_time()
        local_time = time.time()
        self.gps_time_offset = gps_time - local_time
        self.last_valid_offset = self.gps_time_offset  # Update last valid offset
        self.last_sync_time = local_time
        self.logger.info(f"GPS Time: {gps_time:.3f}, Local Time: {local_time:.3f}, Offset: {self.gps_time_offset:.3f} sec")

    def get_synced_time(self):
        return time.time() + self.gps_time_offset