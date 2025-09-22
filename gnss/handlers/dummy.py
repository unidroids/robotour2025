class DummyHandler:
    def __init__(self):
        self.count = 0

    def handle(self, msg_class, msg_id, payload):
        self.count += 1
        if self.count % 100 == 0: 
            print(f"[DUMMY] msg_class=0x{msg_class:02X} msg_id=0x{msg_id:02X} payload_len={len(payload)} count={self.count}")

    def get_count(self):
        return self.count
