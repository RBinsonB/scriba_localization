#!/usr/bin/env python
import numpy as np

class ScribaRobotModel:
    def __init__(self, L):
        """Create robot model

        Keyword arguments:
        L: robot length, distance between the front wheel and rear wheel axis
        """

        self.L = L

    def F(self, state, motion):
        """Update function of the robot position

        Keyword arguments:
        state: current robot state, numpy.array([x, y, theta])
        motion: robot motion, numpy.array([phi, dwf])
            phi: steer angle of the front wheel (in rad)
            dwf: traveled distance of the front wheel (in m)
        """
        x = state[0]
        y = state[1]
        theta = state[2]

        phi = motion[0]
        dwf = motion[1]

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # If driving in a straight line
        if phi == 0.0:
            omega = 0.0
            dx = dwf
            dy = 0.0
        else:
            cos_phi = np.cos(phi)
            sin_phi = np.sin(phi)
            omega = (dwf*sin_phi)/self.L

            dx = self.L * (cos_phi/sin_phi) * np.sin(omega)
            dy = self.L * (cos_phi/sin_phi) * (1-np.cos(omega))

        print("F:")
        print(np.array([x + (cos_theta * dx - sin_theta * dy),
                         y + (sin_theta * dx + cos_theta * dy),
                         (theta + omega) % 2*np.pi]))

        return np.array([x + (cos_theta * dx - sin_theta * dy),
                         y + (sin_theta * dx + cos_theta * dy),
                         (theta + omega)])
                         #(theta + omega) % 2*np.pi]) # Use modulo to wrap the value to 2 pi max

    def Fu(self, state, motion):
        """Robot model jacobian to motion vector

        Could be generated with a symbolic engine like casadi.
        """

        x = state[0]
        y = state[1]
        theta = state[2]

        phi = motion[0]
        dwf = motion[1]

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        # If driving in a straight line
        if phi == 0.0:
            return np.array([0, cos_theta,
                             0, sin_theta,
                             0, 0]).reshape(3,2)
        else:
            cos_phi = np.cos(phi)
            sin_phi = np.sin(phi)

            omega = (dwf*sin_phi)/self.L
            sin_omega = np.sin(omega)
            cos_omega = np.cos(omega)
            elem1 = theta + omega
            elem2 = self.L*cos_omega
            elem3 = dwf*cos_omega*cos_phi**2
            elem4 = dwf*sin_omega*cos_phi**2
            
            return np.array([np.cos(elem1)*cos_phi,  (elem2*sin_theta - self.L*sin_theta + self.L*sin_omega*cos_theta - elem3*cos_theta*sin_phi + elem4*sin_phi*sin_theta)/(cos_phi**2 - 1),
                             np.sin(elem1)*cos_phi, -(elem2*cos_theta - self.L*cos_theta - self.L*sin_omega*sin_theta + elem3*sin_phi*sin_theta + elem4*cos_theta*sin_phi)/(cos_phi**2 - 1),
                             sin_phi/self.L, (dwf*cos_phi)/self.L]).reshape(3,2)


    def Fx(self, state, motion):
        """Robot model jacobian to state vector

        Could be generated with a symbolic engine like casadi.
        """

        x = state[0]
        y = state[1]
        theta = state[2]

        phi = motion[0]
        dwf = motion[1]

        # If driving in a straight line
        if phi == 0.0:
            return np.array([1, 0, -np.sin(theta)*dwf,
                             0, 1,  np.cos(theta)*dwf,
                             0, 0, 1]).reshape(3,3)
        else:
            elem1 = (self.L*theta + dwf*np.sin(phi))/self.L
            tan_phi = np.tan(phi)


            return np.array([1, 0, (self.L*(np.cos(elem1) - np.cos(theta)))/tan_phi,
                             0, 1, (self.L*(np.sin(elem1) - np.sin(theta)))/tan_phi,
                             0, 0, 1]).reshape(3,3)
     