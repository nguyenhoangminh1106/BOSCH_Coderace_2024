from time import time as now


class Stopwatch(object):
    def __init__(self, name: str):
        self.name = name
        self.now = None

    def __enter__(self):
        self.now = now()

    def __exit__(self, exception_type, exception_value, traceback):
        dur = (now() - self.now)
        print(f"Module {self.name} ran in {dur:.3f}s")
