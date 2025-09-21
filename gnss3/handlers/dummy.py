class DummyHandler:
    def __init__(self):
        self.count = 0

    def handle(self, msg_class, msg_id, payload):
        self.count += 1

    def get_count(self):
        return self.count
