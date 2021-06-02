#!/usr/bin/env python
import cv2
import numpy as np

def undistord_fisheye(img, K, D, dim):
    '''
    Rectify an image using a fisheye (equidistant) camera model
    img: raw source image
    K: intrinsic camera matrix (3x3)
    D: vector of distortion coefficients (k1,k2,k3,k4), fisheye model
    dim: image dimension on the format (width, height)
    '''

    # Generate look-up tables for remapping the camera image
    mapx, mapy = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, dim, cv2.CV_16SC2)

    # Remap the original image to a new image
    return cv2.remap(img, mapx, mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def decompose_homography(A, H):
    '''
    Decompose homography matrix using camera matrix to get camera frame.
    Returns rotation matrix and translation vector from camera frame to surface image frame
    A: intrinsic camera matrix (3x3)
    H: homography matrix (3x3)
    '''
    H = np.transpose(H)
    h1 = H[0]
    h2 = H[1]
    h3 = H[2]

    Ainv = np.linalg.inv(A)

    L = 1 / np.linalg.norm(np.dot(Ainv, h1))

    r1 = L * np.dot(Ainv, h1)
    r2 = L * np.dot(Ainv, h2)
    r3 = np.cross(r1, r2)

    T = L * np.dot(Ainv, h3)

    R = np.array([[r1], [r2], [r3]])
    R = np.reshape(R, (3, 3))
    U, S, V = np.linalg.svd(R, full_matrices=True)

    U = np.matrix(U)
    V = np.matrix(V)
    R = U * V

    return (R, T)
