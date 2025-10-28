__all__ = [
    "DummyHandler",
]

class DummyHandler:
    def __init__(self, every=10):
        self.count = 0
        self.every = every

    def handle(self, message_bytes: bytes):
        self.count += 1
        if self.count % self.every == 0:
            print(f"#{self.count}:", message_bytes.decode(errors='ignore').strip())