class DAQProtocol:
    def __init__(self):
        self.version = 1

    def encode(self, packet):
        return packet

    def decode(self, payload):
        return payload
