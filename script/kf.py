import matplotlib.pyplot as plot
import numpy as np

delta = 0.01
DEBUG = False

def print_if(str):
    if DEBUG:
        print(str)

class System():
    def __init__(self, x0=np.zeros([2,1]), dist=False, noise=False):
        self.A = np.array([[0, 1],[-2, -2]])
        self.B = np.array([[0],[1]])
        self.C = np.array([1, 0])
        self.D = np.zeros(1)
        self.x = np.zeros([2,1])

        self.x0 = x0
        self.dist = dist
        self.noise = noise
        self.y = self.output(u=0)

        
    def next(self, u = 0, dt = delta):
        if self.dist:
            mu, sigma = 0, 0.1 # mean and standard deviation
            d = np.ones([2,1]) * np.random.normal(mu, sigma, 1)
        else:
            d = np.zeros([2,1])
        x0_dot = np.dot(self.A, self.x0) + np.dot(self.B, u) + d
        self.x0 = self.integrate(x0_dot, dt)
        self.y = self.output(u)
        # print message 
        print_if(f'x0=\n{self.x0}')
        print_if(f'A=\n{self.A}')
        print_if(f'self.A * self.x0 =\n{np.dot(self.A, self.x0)}')
        print_if(f'self.B * u =\n{self.B * u}')
        print_if(f'x0_dot =\n{x0_dot}')
        print_if(f'x0 =\n{self.x0}')
        print_if(f'y =\n{self.y}')

    def output(self, u = 0):
        if self.noise:
            mu, sigma = 0, 0.01 # mean and standard deviation
            w = np.ones([1]) * np.random.normal(mu, sigma, 1)
        else:
            w = np.zeros([1])
        return np.dot(self.C, self.x0) + np.dot(self.D, u) + w

    def integrate(self, x0_dot = np.zeros([2,1]), dt = delta):
        return self.x0 + x0_dot * dt

if __name__ == "__main__":
    print ("start")
    s = System(x0 = np.zeros([2,1]))
    s1 = System(x0 = np.zeros([2,1]), dist=False, noise=True)
    tmax = 10
    time = range(0, round(tmax/delta + delta))
    u = 1
    y = []
    y1 = []
    for i in range(0, len(time)):
        s.next(u, dt = delta)
        s1.next(u, dt = delta)
        y.append(s.y)
        y1.append(s1.y)
    plot.plot(time, y, time, y1)
    plot.grid(True)
    plot.show()
