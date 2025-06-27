import socket
import threading
import pickle  # or json, depending on format
import json


class LeaderUDPListener:
    def __init__(self, port=9999, use_pickle=False):
        self.port = port
        self.use_pickle = use_pickle
        self.running = False
        self.thread = None
        self.latest_data = None

        self.lock = threading.Lock()

    def _listen(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', self.port))
        print(f"[UDP Listener] Listening on port {self.port} using {'Pickle' if self.use_pickle else 'JSON'}")

        while self.running:
            try:
                data, _ = sock.recvfrom(2048)
                msg = pickle.loads(data) if self.use_pickle else json.loads(data.decode())
                self.latest_data = msg
                # print("[Follower] Received leader update:", msg)

            except Exception as e:
                print("[UDP Listener] Failed to decode:", e)

    # def _listen(self):
    #     print("[UDP Listener] Listening on port", self.port)
    #     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #     sock.bind(('0.0.0.0', self.port))
    #     while True:
    #         data, _ = sock.recvfrom(1024)
    #         print("[UDP Listener] Raw data:", data)

    #         try:
    #             self.latest_data = data.decode('utf-8')
    #         except Exception as e:
    #             print("[UDP Listener] Failed to decode:", e)


    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._listen, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def get_latest(self):
        return self.latest_data
