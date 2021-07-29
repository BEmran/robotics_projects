import matplotlib.pyplot as plot
import numpy as np
from numpy.linalg import inv

delta = 0.01
DEBUG = False

def print_if(str):
    if DEBUG:
        print(str)

class System():
    def __init__(self, x0=np.zeros([2,1]), dist=False, noise=False):
        self.A = np.array([[0, 1],[-2, -2]])
        self.B = np.array([[0],[1]])
        self.C = np.array([[1, 0]])
        self.D = np.zeros(1)

        self.x = x0
        self.x0 = x0
        self.dist = dist
        self.noise = noise
        self.y = self.output(u=0)
        
    def next(self, u = 0, dt = delta):
        if self.dist:
            mu, sigma = 0, 1 # mean and standard deviation
            d = np.ones([2, 1]) * np.random.normal(mu, sigma, 1)
        else:
            d = np.zeros([2, 1])
        x_dot = np.dot(self.A, self.x) + np.dot(self.B, u) + d
        self.x = self.integrate(x_dot, dt)
        self.y = self.output(u)
        # print message 
        print_if(f'x0=\n{self.x}')
        print_if(f'A=\n{self.A}')
        print_if(f'self.A * self.x =\n{np.dot(self.A, self.x)}')
        print_if(f'self.B * u =\n{self.B * u}')
        print_if(f'x_dot =\n{x_dot}')
        print_if(f'x =\n{self.x}')
        print_if(f'y =\n{self.y}')

    def output(self, u = 0):
        if self.noise:
            mu, sigma = 0, 0.01 # mean and standard deviation
            w = np.ones([1, 1]) * np.random.normal(mu, sigma, 1)
        else:
            w = np.zeros([1, 1])
        return np.dot(self.C, self.x) + np.dot(self.D, u) + w

    def integrate(self, x_dot = np.zeros([2,1]), dt = delta):
        return self.x + x_dot * dt
        
class KF():
    def __init__(self, x0=np.zeros([2,1])):
        self.A = np.array([[0, 1],[-2, -2]])
        self.B = np.array([[0],[1]])
        self.C = np.array([[1, 0]])
        self.D = np.zeros(1)

        self.x = x0
        self.x0 = x0
        self.y = self.output(u=0)
        self.P = np.array([[0.0, 0], [0, 0.0]])
        self.Q = np.array([[0.001, 0], [0, 0.001]])
        self.R = np.array([0.001])
        self.K = np.zeros([2, 1])

    def predict(self, u=0, dt = delta):
        # predict state
        x_dot = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.x = self.integrate(x_dot, dt)

        # predict process coverience
        self.P = np.dot(np.dot(self.A , self.P), self.A.T) + self.Q


    def update(self, z=0):
        # Calculating the Kalman Gain
        S = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        # print(S)
        self.K = np.dot(np.dot(self.P, self.C.T), inv(S))
        
        # Update the State Matrix
        e = z - np.dot(self.C, self.x)
        self.x = self.x + np.dot(self.K, e)

        # Update Process Covariance Matrix
        self.P = np.dot(np.identity(len(self.K)) - np.dot(self.K, self.C), self.P)

    def step(self, u=0, z=0, dt = delta):
        self.predict(u, dt)
        self.update(z)
        self.y = self.output(u)
        return self.x

    def integrate(self, x_dot = np.zeros([2,1]), dt = delta):
        return self.x + x_dot * dt

    def output(self, u = 0):
        return np.dot(self.C, self.x) + np.dot(self.D, u)

if __name__ == "__main__":
    print ("start")
    real = System(x0 = np.zeros([2,1]))
    # sys1 = System(x0 = np.zeros([2,1]), dist=False, noise=False)
    kf = KF(x0 = np.zeros([2,1]))

    tmax = 10
    time = range(0, round(tmax/delta + delta))
    u = 1
    yr = []
    y1 = []
    ye = []
    for i in range(0, len(time)):
        real.next(u, dt = delta)
        # sys1.next(u, dt = delta)

        yr.append(real.y[0][0])
        # y1.append(sys1.y[0][0])
        measurment = real.y[0][0] + np.random.normal(0, 0.1, 1)
        y1.append(measurment)

        kf.step(u, dt = delta, z=measurment)
        ye.append(kf.y[0][0])
    
    plot.plot(time, yr, marker='o', color='b')
    plot.plot(time, y1, color='r')
    plot.plot(time, ye, color='g')
    plot.grid(True)
    plot.show()
