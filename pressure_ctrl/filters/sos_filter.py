import numpy as np

class LiveFilter:
    def process(self, x):
        if np.isnan(x):     # skip value if it is invalid
            return(x)
        else:
            return self._process(x)
    
    def __call__(self, x):
        return self.process(x)
    
    def _process(self, x):      # for use if we want to try other filters
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