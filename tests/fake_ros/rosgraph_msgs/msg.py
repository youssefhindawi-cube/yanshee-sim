import time


class _Stamp(object):
    def __init__(self):
        self.t = time.time()

    def to_sec(self):
        return self.t


class _Header(object):
    def __init__(self):
        self.stamp = _Stamp()


class Log(object):
    INFO = 2

    def __init__(self, msg="", name="/hal", level=2):
        self.msg, self.name, self.level, self.function, self.header = msg, name, level, "fake", _Header()
