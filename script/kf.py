import matplotlib.pyplot as plot
import numpy as np
from numpy.linalg import inv

delta = 0.01
DEBUG = False


def print_if(str):
    if DEBUG:
        print(str)


def to_column(input=np.array, nrow=int):
    input = np.array(input, ndmin=1)  # convert input to array
    if input.shape[0] != nrow:
        raise TypeError(
            f'the passed input has a wrong length. It has a length of {len(input)} and it should be {nrow}')
    # reshape input to 2d array with the correct shape => nrowx1
    return input.reshape(nrow, 1)


class System():
    def __init__(self, x0=np.zeros([2, 1]), dist=False, noise=False):
        A = np.array([[0, 1], [-2, -2]])
        B = np.array([[0], [1]])
        # deserialization
        self.A = np.identity(2) + A*delta
        self.B = B*delta
        self.C = np.array([[1, 0]])
        self.D = np.zeros([1, 1])

        # check matrices sizes
        if self.A.shape[0] != self.A.shape[1]:
            raise TypeError(
                f'the matrix A is not square. It has size of {self.A.shape}')
        if self.A.shape[0] != self.B.shape[0] != self.C.shape[1]:
            raise TypeError(
                f'the matrix A, B and C don\'t have the same row size. A has size of {self.A.shape}'
                f' while B has size of {self.B.shape} and C has size of {self.C.shape}')
        if self.B.shape[1] != self.D.shape[1]:
            raise TypeError(
                f'the matrix B and D don\'t have the same column size. B has size of {self.B.shape}'
                f' while D has size of {self.D.shape}')
        if self.C.shape[0] != self.D.shape[0]:
            raise TypeError(
                f'the matrix C and D don\'t have the same row size. C has size of {self.C.shape}'
                f' while D has size of {self.D.shape}')

        self.n_state = self.A.shape[1]
        self.n_input = self.B.shape[1]
        self.n_output = self.C.shape[0]

        self.x = x0
        self.x0 = x0
        self.dist = dist
        self.noise = noise
        # calculate initial output
        self.y = self.__output(u=0)

    def step(self, u=np.array, dt=int):
        u = to_column(u, self.n_input)

        if self.dist:
            mu, sigma = 0, 0.01  # mean and standard deviation
            d = np.ones([self.n_state, 1]) * np.random.normal(mu, sigma, 1)
        else:
            d = np.zeros([self.n_state, 1])

        x_dot = np.dot(self.A, self.x) + np.dot(self.B, u) + d
        self.x = x_dot
        # calculate initial output
        self.y = self.__output(u=0)

        # print message
        print_if(f'x0=\n{self.x}')
        print_if(f'A=\n{self.A}')
        print_if(f'self.A * self.x =\n{np.dot(self.A, self.x)}')
        print_if(f'self.B * u =\n{self.B * u}')
        print_if(f'x_dot =\n{x_dot}')
        print_if(f'x =\n{self.x}')
        print_if(f'y =\n{self.y}')

    def __output(self, u=np.array):
        u = to_column(u, self.n_input)
        if self.noise:
            mu, sigma = 0, 0.01  # mean and standard deviation
            w = np.ones([self.n_output, 1]) * np.random.normal(mu, sigma, 1)
        else:
            w = np.zeros([self.n_output, 1])
        return np.dot(self.C, self.x) + np.dot(self.D, u) + w


class KF():
    def __init__(self, x0=np.array):
        self.F = np.array([[0, 1], [-2., -2.]])
        self.B = np.array([[0], [1]])
        self.H = np.array([[1, 0]])

        self.F = np.identity(2) + self.F*delta
        self.B = self.B*delta

        # check matrices sizes
        if self.F.shape[0] != self.F.shape[1]:
            raise TypeError(
                f'the matrix F is not square. It has size of {self.F.shape}')
        if self.F.shape[0] != self.B.shape[0] != self.H.shape[1]:
            raise TypeError(
                f'the matrix F, B and H don\'t have the same row size. F has size of {self.A.shape}'
                f' while B has size of {self.B.shape} and H has size of {self.C.shape}')

        self.n_state = self.F.shape[1]
        self.n_input = self.B.shape[1]
        self.n_measurment = self.H.shape[0]

        self.x = x0
        self.P = np.identity(2) * 1
        self.Q = np.identity(2) * 2
        self.R = np.identity(1) * 10
        self.K = np.zeros([2, 1])

    def __predict(self, u=np.array, dt=delta):
        # state estimtion
        x_dot = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.x = x_dot
        # process coverience
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose()) + self.Q

    def __update(self, measurment=np.array):
        # Innovation
        S = np.dot(np.dot(self.H, self.P), self.H.transpose()) + self.R
        # Calculating the Kalman Gain
        self.K = np.dot(np.dot(self.P, self.H.transpose()), inv(S))
        # measurement pre-fit residual
        y = np.dot(self.H, self.x)
        e = measurment - y
        # Update state estimation
        self.x = self.x + np.dot(self.K, e)
        # Updated estimate covariance
        self.P = np.dot(np.identity(self.n_state) -
                        np.dot(self.K, self.H), self.P)
        # self.P = self.P - np.dot(np.dot(self.K, self.H), self.P)

    def step(self, u=0, measurment=0, dt=delta):
        u = to_column(u, self.n_input)
        measurment = to_column(measurment, self.n_measurment)
        self.__predict(u, dt)
        self.__update(measurment)
        return self.x


if __name__ == "__main__":
    print("start")
    x0 = np.zeros([2, 1])
    x0_est = np.ones([2, 1])*-1
    sys = System(x0=x0, dist=False, noise=False)
    kf = KF(x0=x0_est)

    tmax = 10
    time = range(0, round(tmax/delta + delta))
    u = 1
    yr = []
    ym = []
    ye = []
    K1 = []
    K2 = []
    e = []

    for i in range(0, len(time)):
        if i > 100:
            u = 1
        sys.step(u, dt=delta)
        real = sys.y[0][0]
        yr.append(real)

        measurment = real + np.random.normal(0, 0.1, 1)
        ym.append(measurment)

        kf.step(u, dt=delta, measurment=measurment)
        ye.append(kf.x[0])

        K1.append(kf.K[0])
        K2.append(kf.K[1])
        e.append(real-ye[-1])

    print(f'rms = {np.linalg.norm(e)}')

    fig, axis = plot.subplots(3)
    axis[0].plot(time, yr, marker='o', color='b', label='real')
    axis[0].plot(time, ym, color='r', label='noise')
    axis[0].plot(time, ye, color='g', label='filtred')
    axis[0].grid(True)
    axis[0].legend()

    axis[1].plot(time, K1, color='r', label='kalman gain 1')
    axis[1].plot(time, K2, color='g', label='kalman gain 2')
    axis[1].grid(True)
    axis[1].legend()

    axis[2].plot(time, e, color='k', label='error (real-filtered)')
    axis[2].grid(True)
    axis[2].legend()
    plot.show()
