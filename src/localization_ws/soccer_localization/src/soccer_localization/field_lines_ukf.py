#!/usr/bin/env python3
import math
import os
from math import atan2, cos, nan, sin, sqrt, tan
from typing import Optional
import cv2
import numpy as np
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.stats import plot_covariance
from nav_msgs.msg import OccupancyGrid
from soccer_common.transformation import Transformation
from soccer_common.utils import wrapToPi

# Adapted from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb

class FieldLinesUKF:
    def __init__(self):
        points_fn = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2, kappa=0, subtract=self.residual_x)
        self.ukf = UKF(
            dim_x=3,
            dim_z=3,
            fx=self.move,
            hx=self.Hx,
            dt=0.01,
            points=points_fn,
            x_mean_fn=self.state_mean,
            z_mean_fn=self.z_mean,
            residual_x=self.residual_x,
            residual_z=self.residual_h,
        )
        self.ukf.x = np.array([-4, -3.15, 1.57])  # Initial state
        self.ukf.P = np.diag([0.0004, 0.0004, 0.002])  # Initial covariance (2cm, 2cm, 3 degrees)
        
        self.R_walking = np.diag([4, 2, 0.1])
        self.R_localizing = np.diag([0.9, 0.9, 0.1])
        self.R_ready = np.diag([0.1, 0.1, 0.1])
        self.ukf.R = self.R_walking  # Noise from measurement updates (x, y, theta)
        
        self.Q_walking = np.diag([9e-5, 9e-5, 5e-4])
        self.Q_localizing = np.diag([9e-5, 9e-5, 5e-4])
        self.Q_ready = np.diag([9e-5, 9e-5, 5e-4])
        self.ukf.Q = self.Q_walking  # Noise from navigation movements

    def map_update(self, map: OccupancyGrid):
        self.map = map

    def move(self, x: [float], dt: float, u: [float]) -> [float]:
        pos = np.array([[cos(x[2]), -sin(x[2])], [sin(x[2]), cos(x[2])]]) @ np.array([u[0] * dt, u[1] * dt]) + np.array(x[0:2])
        return [pos[0], pos[1], x[2] + u[2] * dt]

    def residual_h(self, a, b):
        y = a - b
        y[2] = wrapToPi(y[2])
        return y

    def residual_x(self, a, b):
        y = a - b
        y[2] = wrapToPi(y[2])
        return y

    def Hx(self, x):
        """
        takes a state variable and returns the measurement
        that would correspond to that state
        :param x: offset transform of robot from field
        :return: an array of distance and bearings in relation to the robot
        """
        return x

    def state_mean(self, sigmas, Wm):
        x = np.zeros(3)
        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 2]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 2]), Wm))
        x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
        x[1] = np.sum(np.dot(sigmas[:, 1], Wm))
        x[2] = atan2(sum_sin, sum_cos)
        return x

    def z_mean(self, sigmas, Wm):
        z_count = sigmas.shape[1]
        x = np.zeros(z_count)
        x[0] = np.mean(sigmas[:, 0])
        x[1] = np.mean(sigmas[:, 1])
        x[2] = np.arctan2(np.sum(np.sin(sigmas[:, 2])) / len(sigmas), np.sum(np.cos(sigmas[:, 2])) / len(sigmas))
        return x

    # ===== FIXED METHODS =====
    
    def ensure_positive_definite(self, P):
        """
        Ensure covariance matrix remains positive definite
        Fixes numerical stability issues that cause LinAlgError
        """
        # Make symmetric (fix floating point asymmetry)
        P = (P + P.T) / 2.0
        
        # Add small regularization to diagonal
        epsilon = 1e-6
        P += np.eye(len(P)) * epsilon
        
        # Ensure diagonal is positive
        for i in range(len(P)):
            if P[i, i] < epsilon:
                P[i, i] = epsilon
        
        # Clip extreme values (prevent unbounded growth)
        P = np.clip(P, -100.0, 100.0)
        
        return P

    def predict(self, u, dt):
        """Modified predict with stability checks"""
        assert dt >= 0
        
        # Ensure covariance is positive definite BEFORE predict
        self.ukf.P = self.ensure_positive_definite(self.ukf.P)
        
        try:
            self.ukf.predict(dt=dt, u=u)
        except np.linalg.LinAlgError as e:
            print(f"UKF predict failed: {e}, resetting covariance")
            # Reset to safe values
            self.ukf.P = np.diag([0.1, 0.1, 0.05])
            return
        except Exception as e:
            print(f"Unexpected error in predict: {e}")
            return
        
        # Ensure covariance is positive definite AFTER predict
        self.ukf.P = self.ensure_positive_definite(self.ukf.P)
        
        assert not math.isnan(self.ukf.x[0])

    def update(self, z, transform_confidence):
        """Modified update with stability checks"""
        assert not any((math.isnan(z[i]) for i in range(0, 3)))
        
        R = np.copy(self.ukf.R)
        R[0, 0] = R[0, 0] / max(0.001, transform_confidence[0] ** 2)
        R[1, 1] = R[1, 1] / max(0.001, transform_confidence[1] ** 2)
        R[2, 2] = R[2, 2] / max(0.001, transform_confidence[2] ** 2)
        
        # Ensure covariance is positive definite BEFORE update
        self.ukf.P = self.ensure_positive_definite(self.ukf.P)
        
        try:
            self.ukf.update(z, R)
        except np.linalg.LinAlgError as e:
            print(f"UKF update failed: {e}, skipping measurement")
            return
        except Exception as e:
            print(f"Unexpected error in update: {e}")
            return
        
        # Ensure covariance is positive definite AFTER update
        self.ukf.P = self.ensure_positive_definite(self.ukf.P)
        
        assert not math.isnan(self.ukf.x[0])

    def draw_covariance(self):
        plot_covariance((self.ukf.x[0], self.ukf.x[1]), self.ukf.P[0:2, 0:2], std=1, facecolor="k", alpha=0.1)