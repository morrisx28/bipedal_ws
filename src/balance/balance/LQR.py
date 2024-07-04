import math
import time
from Com import Body
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import inv, eig


class InvertedPendulumLQR:
    def __init__(self, hip=0, knee=0, l_bar=None, M=(0.0452+0.0225), m=1.228, g=9.8, Q=None, R=None, delta_t=1/60, sim_time=15.0, show_animation=True):
        self.hip = hip - 90
        self.knee = self.hip - knee
        self.com = Body(self.hip, self.knee)
        self.l_bar = self.com.length  # length of bar
        theta3 = self.com.theta3
        self.theta_bias = math.radians(knee) - math.radians(hip) - theta3
        # self.l_bar = 0.09
        print('hip', hip, 'knee', knee)
        print('l_bar:', self.l_bar)
        print('theta_bias:', math.degrees(self.theta_bias))
        # mass of the cart [kg]self.R = R if R is not None else np.diag([0.1])  # input cost matrix
        self.M = M
        self.m = m  # mass of the pendulum [kg]
        self.g = g  # gravity [m/s^2]
        self.nx = 4  # number of states
        self.nu = 1  # number of inputs
        self.wheel_r = 0.06152/2
        # state cost matrix , best in IsaacSim
        self.Q = Q if Q is not None else np.diag([0.0, 2.5, 200.0, 150.0])
        self.R = R if R is not None else np.diag([1e-6])  # input cost matrix

        # self.Q = Q if Q is not None else np.diag([0.1, 0.001, 30.0, 0.0])  # state cost matrix
        # self.R = R if R is not None else np.diag([0.001])  # input cost matrix

        self.delta_t = delta_t  # time tick [s]
        self.sim_time = sim_time  # simulation time [s]

        self.show_animation = show_animation

        self.A, self.B = self.get_model_matrix()
        self.K, _, _ = self.dlqr(self.A, self.B, self.Q, self.R)
        print("K", self.K)

    def simulation(self, x, u):
        A, B = self.get_model_matrix()
        x = A @ x + B @ u

        return x

    def solve_DARE(self, A, B, Q, R, maxiter=150, eps=0.01):
        """
        Solve a discrete time Algebraic Riccati equation (DARE)
        """
        P = Q

        for i in range(maxiter):
            Pn = A.T @ P @ A - A.T @ P @ B @ \
                inv(R + B.T @ P @ B) @ B.T @ P @ A + Q
            if (abs(Pn - P)).max() < eps:
                break
            P = Pn

        return Pn

    def dlqr(self, A, B, Q, R):
        """
        Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        """

        # first, try to solve the ricatti equation
        P = self.solve_DARE(A, B, Q, R)

        # compute the LQR gain
        K = inv(B.T @ P @ B + R) @ (B.T @ P @ A)
        eigVals, eigVecs = eig(A - B @ K)
        return K, P, eigVals

    def lqr_control(self, x):
        start = time.time()
        # K, _, _ = self.dlqr(self.A, self.B, self.Q, self.R)
        # K = np.array([[-0.0923, -3.0287, 46.1389, 3.4110]]) #our param in matlab get K
        # K = np.array([[-0.0205, -2.0707, 35.212, 11.767]]) #our param in matlab get K
        
        u = -self.K @ x
        # elapsed_time = time.time() - start
        # print(f"calc time:{elapsed_time:.6f} [sec]")
        return u

    def get_model_matrix(self):
        Jz = (1/3)*self.m*self.l_bar**2
        I = (1/2)*self.M*self.wheel_r**2
        Q_eq = Jz*self.m + (Jz+self.m*self.l_bar*self.l_bar) * \
            (2*self.M+(2*I)/(self.wheel_r**2))
        A_23 = -(self.m**2)*(self.l_bar**2)*self.g / Q_eq
        A_43 = self.m*self.l_bar*self.g * \
            (self.m+2*self.M+(2*I/(self.wheel_r**2)))/Q_eq
        A = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, A_23, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, A_43, 0.0]
        ])
        # print('A=',A)
        A = np.eye(self.nx) + self.delta_t * A

        B_21 = (Jz+self.m*self.l_bar**2+self.m *
                self.l_bar*self.wheel_r)/Q_eq/self.wheel_r
        B_41 = -((self.m*self.l_bar/self.wheel_r)+self.m +
                 2*self.M+(2*I/(self.wheel_r**2)))/Q_eq
        B = np.array([
            [0.0],
            [2*B_21],
            [0.0],
            [2*B_41]
        ])
        # print('B=',B)
        B = self.delta_t * B

        return A, B
    
    

    def plot_cart(self, xt, theta):
        cart_w = 1.0
        cart_h = 0.5
        radius = 0.1

        cx = np.array([-cart_w / 2.0, cart_w / 2.0, cart_w /
                       2.0, -cart_w / 2.0, -cart_w / 2.0])
        cy = np.array([0.0, 0.0, cart_h, cart_h, 0.0])
        cy += radius * 2.0

        cx = cx + xt

        bx = np.array([0.0, self.l_bar * math.sin(-theta)])
        bx += xt
        by = np.array([cart_h, self.l_bar * math.cos(-theta) + cart_h])
        by += radius * 2.0

        angles = np.arange(0.0, math.pi * 2.0, math.radians(3.0))
        ox = np.array([radius * math.cos(a) for a in angles])
        oy = np.array([radius * math.sin(a) for a in angles])

        rwx = np.copy(ox) + cart_w / 4.0 + xt
        rwy = np.copy(oy) + radius
        lwx = np.copy(ox) - cart_w / 4.0 + xt
        lwy = np.copy(oy) + radius

        wx = np.copy(ox) + bx[-1]
        wy = np.copy(oy) + by[-1]

        plt.plot(cx.flatten(), cy.flatten(), "-b")
        plt.plot(bx.flatten(), by.flatten(), "-k")
        plt.plot(rwx.flatten(), rwy.flatten(), "-k")
        plt.plot(lwx.flatten(), lwy.flatten(), "-k")
        plt.plot(wx.flatten(), wy.flatten(), "-k")
        plt.title(f"x: {xt:.2f} , theta: {math.degrees(theta):.2f}")

        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        plt.axis("equal")

    def main(self):
        x0 = np.array([
            [0.0],
            [0.0],
            [math.radians(10)],
            [0.0]
        ])

        x = np.copy(x0)
        time_elapsed = 0.0

        while self.sim_time > time_elapsed:
            # while True:
            time_elapsed += self.delta_t

            # calculate control input
            u = self.lqr_control(x)
            # print(u,'N')
            print(x[0], '\n')
            # simulate inverted pendulum cart
            x = self.simulation(x, u)

            if self.show_animation:
                plt.clf()

                px = float(x[0, 0])
                theta = float(x[2, 0])
                self.plot_cart(px, theta)
                plt.xlim([-5.0, 2.0])
                plt.pause(0.001)

        print("Finish")
        print(
            f"x={float(x[0, 0]):.2f} [m] , theta={math.degrees(x[2, 0]):.2f} [deg]")
        if self.show_animation:
            plt.show()


if __name__ == '__main__':
    lqr = InvertedPendulumLQR(72.5, 125)
    lqr.main()
