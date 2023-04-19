from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np
from math import cos, sin, atan2


def Hx(x_state):
    return np.array(x_state[:3])

def fx(x, dt, u):
    x_prior = np.zeros(5)

    x_prior[0] = x[0] + u[0] * cos(x[2]) * dt
    x_prior[1] = x[1] + u[0] * sin(x[2]) * dt
    x_prior[2] = x[2] + u[1]
    x_prior[3] = u[0]
    x_prior[4] = u[1]

    return x_prior

def state_mean(sigmas, Wm):
    x = np.zeros(5)
    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = atan2(sum_sin, sum_cos)
    x[3] = np.sum(np.dot(sigmas[:, 3], Wm))
    x[4] = np.sum(np.dot(sigmas[:, 4], Wm))
    return x

def z_mean(sigmas, Wm):
    x = np.zeros(3)
    sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
    sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
    x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
    x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
    x[2] = atan2(sum_sin, sum_cos)
    return x

def normalize_angle(x):
    x = x % (2 * np.pi)
    # force in range [0, 2 pi)
    if x > np.pi:
        x -= 2 * np.pi
    return x

def residual_state(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y

def residual_z(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])
    return y









