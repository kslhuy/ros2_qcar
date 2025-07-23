import threading

class ControlThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self._kill_thread = threading.Event()
        pass

    def run(self):
        
        pass

    def stop(self):
        self._kill_thread.set()
        pass

    def should_stop(self):
        return self._kill_thread.is_set()

    def WriteControlInput(self, ControlInputAPI):

        pass
    
    def ReadCarState(self, FollowerID, LeaderID):

        pass