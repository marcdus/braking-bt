import numpy as np
import scipy
import matplotlib.pyplot as plt

class LiveFilter:
    def process(self, x):
        if np.isnan(x):
            return(x)
        else:
            return self._process(x)
    
    def __call__(self, x):
        return self.process(x)
    
    def _process(self, x):
        raise NotImplementedError("derived processing class is missing")

class LiveSosFilter(LiveFilter):
    def __init__(self, sos):
        self.sos = sos

        self.n_sections = sos.shape[0]              # find out the shape of the second order section filter
        self.state = np.zeros((self.n_sections, 2)) # create empty "filter template"
    
    def _process(self, x):
        for s in range(self.n_sections):
            b0, b1, b2, a0, a1, a2 = self.sos[s, :]

            y = b0*x + self.state[s, 0]
            self.state[s, 0] = b1*x - a1*y + self.state[s, 1]
            self.state[s, 1] = b2*x - a2*y
            x = y
        return y

import csv

with open("examples/pressure.csv", encoding='utf-8-sig') as file_name:
    data = np.array(list(csv.reader(file_name)), dtype=float)


sos = scipy.signal.iirfilter(1.0, Wn=2.0, fs=20, btype="low",
                            ftype="butter", output="sos")
live_sosfilter = LiveSosFilter(sos)
y_live_sosfit = [live_sosfilter(y) for y in data]


a = 22100
b = 22400
plt.figure(figsize=[6.4, 2.4])
plt.plot(y_live_sosfit[a:b], label="filtered")
plt.plot(data[a:b])
plt.legend()
plt.show()