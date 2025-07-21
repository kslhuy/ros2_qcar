import socket
import threading
import json
import time

from qvl.qcar2 import QLabsQCar2

HOST = '0.0.0.0'
PORT = 5005
UPDATE_RATE = 0.05  # 20 Hz

class QCar2Sim:
    def __init__(self, qcar2: QLabsQCar2, id: int, lock: threading.Lock):
        self.qcar2 = qcar2
        self.id = id
        self.port = PORT + self.id
        self.last_cmd = {"motor": 0.0, "steering": 0.0}
        self.lock = lock

    def start(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, self.port))
            s.listen(1)
            print(f"[TCP Server] Listening on {HOST}:{self.port}")
            conn, addr = s.accept()
            print(f"[TCP Server] Connected by {addr}")

            recv_thread = threading.Thread(target=self.receive_commands, args=(conn,))
            recv_thread.start()
            send_sensor_data_loop = threading.Thread(target=self.send_sensor_data, args=(conn,))
            send_sensor_data_loop.start()
            

    def send_sensor_data(self, conn: socket.socket):
        while True:
            try:
                with self.lock:
                    try:
                        # Apply last command                
                        self.qcar2.set_velocity_and_request_state(
                            forward=self.last_cmd["motor"], 
                            turn=self.last_cmd["steering"], 
                            headlights=False,
                            leftTurnSignal=False, 
                            rightTurnSignal=False, 
                            brakeSignal=False, 
                            reverseSignal=False
                        )
                    except Exception as e:
                        print(f"[QLabs] set_velocity_and_request_state error: {e}")
                        continue
                    
                    try:
                        # Get sensor data
                        _, position, rotation, _ = self.qcar2.get_world_transform()
                    except Exception as e:
                        print(f"[QLabs] get_world_transform error: {e}")
                        continue
                    
                    try:
                        success, angles, distances = self.qcar2.get_lidar()
                        if not success:
                            print("[QLabs] Lidar data fetch failed.")
                        lidar = {
                            "angles": angles.tolist() if success else [],
                            "distances": distances.tolist() if success else []
                        } 
                    except Exception as e:
                        print(f"[QLabs] get_lidar error: {e}")
                        continue

                data_packet = {
                    "timestamp": time.time(),
                    "position": position,
                    "rotation": rotation,
                    "lidar": lidar
                }

                conn.sendall((json.dumps(data_packet) + "\n").encode())
                time.sleep(UPDATE_RATE)
            except Exception as e:
                print(f"[TCP Server] Error: {e}")
                break

    def receive_commands(self, conn: socket.socket):
        buffer = b""
        while True:
            try:
                data = conn.recv(1024)
                if not data:
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    cmd = json.loads(line.decode())
                    self.last_cmd = cmd
                    
            except Exception as e:
                print(f"[Command Thread] Error: {e}")
                break

if __name__ == "__main__":
    QCar2Sim().start()