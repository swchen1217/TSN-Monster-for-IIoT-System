import time
import threading

def delayNS(ns, t=time.perf_counter_ns):  ## Delay at nano second
    n = t()
    e = n + ns
    while n < e:
        n = t()

class SetIntervalNS:
    def __init__(self, nsInterval, onRun):
        self.ns = nsInterval
        self.onRun = onRun
        self.kStop = False
        self.thread = threading.Thread(target=self.Clock)
        self.thread.start()

    def stop(self):
        self.kStop = False

    def Clock(self):
        while self.kStop is not True:
            self.onRun(self)
            delayNS(self.ns)
