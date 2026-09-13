class UDPConnection:
    def __init__(self, host="127.0.0.1", port=5000):
        self.host = host
        self.port = port

    def connect(self):
        return True

    def disconnect(self):
        return True

    def send(self, payload):
        return payload
