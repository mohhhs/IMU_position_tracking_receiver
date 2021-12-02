"""A file implementing the quaternion class"""

'''[+] this sign means I tested the component and added by me
'''
# getting rid of all sympy since it causes type confilics with numpy
# import sympy as sp
import math
import numpy as np


class Quaternion(object):
    """A class for describing a quaternion."""
    def __init__(self, q0=0, q1=0, q2=0, q3=0):
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def array(self):  # return numpy array
        # douple prantes, to use np.concatenate
        return np.array([[self.q0, self.q1, self.q2, self.q3]])


class Sigma(object):
    '''[+] A class for sigma points'''
    sig_num = 0

    def __init__(self, q0=0, q1=0, q2=0, q3=0):
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        Sigma.sig_num += 1
        self.num = Sigma.sig_num  # to identfy sigma

    def array(self):  # return numpy array
        # douple prantes, to use np.concatenate
        return np.array([[self.q0, self.q1, self.q2, self.q3]])

class Vector(object):
    """A class for describing a vector"""
    def __init__(self, vx=0, vy=0, vz=0):
        self.vx = vx
        self.vy = vy
        self.vz = vz

    def array(self):  # return numpy array
        # douple prantes, to use np.concatenate
        return np.array([[self.vx, self.vy, self.vz]])

    def matrix(self):  # return numpy matrix
            return np.matrix([self.vx, self.vy, self.vz])


def quaternion_average (lq , q): # works, need to cut the loop by beaking for once condtion met
    '''
    [+] lq: list of quaternions motion sig, q is quaternion mean
    returns: qt: quaternion next_mean, ev list of 6 error vectors
    '''
    qt = q
    nr = len(lq)
    qe = [0] * nr # used for motion sig * 6 quatrnions
    ev = [0] * nr # used for error * 6 vectors
    # just for return
    # qt_r = q
    # ev_r = [0] * nr
    epsilon = 0.0001
    pi = np.pi
    temp = Quaternion(0, 0, 0, 0)

    for t in range(1000):
        for i in range(0,nr,1):
            qe[i] = normalise_quaternion(lq[i]) #orginal code has lq[i] =
            qe[i] = quaternion_product(qe[i], inverse_quaternion(qt))  # ordginal code has (lq[i], inverse...
            qs = qe[i].q0  # scaler
            qv = Vector(qe[i].q1, qe[i].q2, qe[i].q3)  #vector
            # without float() it produces TypeError: round() got an unexpected keyword argument 'decimals'
            # without float() qv_norm type is sympy.core.numbers.Float -> float converts to float type
            qv_norm = float(compute_vector_norm(qv))
            qe_norm = float(compute_quaternion_norm(qe[i]))
            # qe_norm = ((qe[i].q0 ** 2 + qe[i].q1 ** 2 + qe[i].q2 ** 2 + qe[i].q3 ** 2) ** 0.5)
            if np.round(qv_norm, 8) == 0:  # round norm vector to 8 decimals
                if np.round(qe_norm, 8) == 0:  # if vector qe is zero, regardless ev is zero
                    ev[i] = Vector(0, 0, 0)
                else:
                    ev[i] = Vector(0, 0, 0)
            if np.round(qv_norm, 8) != 0:
                if np.round(qe_norm, 8) == 0:
                    ev[i] = Vector(0, 0, 0)
                else:
                    temp.q0 = np.log(qe_norm)
                    temp.q1 = (qv.vx/qv_norm) * math.acos(qs/qe_norm)
                    temp.q2 = (qv.vy/qv_norm) * math.acos(qs/qe_norm)
                    temp.q3 = (qv.vz/qv_norm) * math.acos(qs/qe_norm)
                    ev[i] = Vector(2*temp.q1, 2*temp.q2, 2*temp.q3)
                    ev_norm = compute_vector_norm(ev[i])
                    # ev_norm = ((ev[i].vx ** 2 + ev[i].vy ** 2 + ev[i].vz ** 2) ** 0.5)
                    ev[i].vx = ((-pi + np.mod(ev_norm + pi, 2 * pi)) / ev_norm) * ev[i].vx
                    ev[i].vy = ((-pi + np.mod(ev_norm + pi, 2 * pi)) / ev_norm) * ev[i].vy
                    ev[i].vz = ((-pi + np.mod(ev_norm + pi, 2 * pi)) / ev_norm) * ev[i].vz
        # avarage all 6 ev
        e = Vector()
        for j in range(len(ev)):
            e.vx += ev[j].vx
            e.vy += ev[j].vy
            e.vz += ev[j].vz
        e.vx /= len(ev)
        e.vy /= len(ev)
        e.vz /= len(ev)

        temp2 = Quaternion(0, 0, 0, 0)
        temp2.q0 = 0 + 0.00001
        temp2.q1 = (e.vx / 2) + 0.00001
        temp2.q2 = (e.vy / 2) + 0.00001
        temp2.q3 = (e.vz / 2) + 0.00001
        qt = quaternion_product(exp_quaternion(temp2), qt)

        e_norm = compute_vector_norm(e)
        # e_norm = ((e.vx ** 2 + e.vy ** 2 + e.vz ** 2) ** 0.5)
        if e_norm < epsilon:
            return qt, ev



def quaternion_product(p, q):
    """p, q are two quaternions; quaternion product."""

    p0 = p.q0
    p1 = p.q1
    p2 = p.q2
    p3 = p.q3

    q0 = q.q0
    q1 = q.q1
    q2 = q.q2
    q3 = q.q3

    r0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3
    r1 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2
    r2 = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1
    r3 = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0

    r = Quaternion(r0, r1, r2, r3)

    return(r)


def minus_vector(v):
    """v is a vector"""

    return Vector(-v.vx, -v.vy, -v.vz)


def conjugate_quaternion(q):
    """q is a quaternion"""

    q0 = q.q0
    q1 = q.q1
    q2 = q.q2
    q3 = q.q3

    r = Quaternion(q0, -q1, -q2, -q3)

    return(r)

def inverse_quaternion(q):
    """[+] q is a quaternion"""
    l = compute_quaternion_norm(q)
    q0 =  q.q0 / (l ** 2)
    q1 = -q.q1 / (l ** 2)
    q2 = -q.q2 / (l ** 2)
    q3 = -q.q3 / (l ** 2)

    r = Quaternion(q0, q1, q2, q3)

    return(r)

def exp_quaternion(q):
    """[+] q is a quaternion"""
    q0 = q.q0
    q1 = q.q1
    q2 = q.q2
    q3 = q.q3

    lv = ((q1 ** 2 + q2 ** 2 + q3 ** 2) ** 0.5)
    q0_n = math.cos(lv)
    q1_n = (q1 / lv) * math.sin(lv)
    q2_n = (q2 / lv) * math.sin(lv)
    q3_n = (q3 / lv) * math.sin(lv)

    e_q0 = math.exp(q0)
    r = Quaternion(e_q0*q0_n, e_q0*q1_n, e_q0*q2_n, e_q0*q3_n)

    return r

def transform_vector_to_quaternion(v):
    """v is a vector"""

    r = Quaternion(0, v.vx, v.vy, v.vz)

    return(r)


def apply_rotation_on_vector(q, v):
    """q is the quaternion describing the rotation to apply, v is the vector on
    which to apply the rotation"""

    quaternion_v = transform_vector_to_quaternion(v)
    transposed_q = conjugate_quaternion(q)

    r = quaternion_product(quaternion_product(q, quaternion_v), transposed_q) # and this is the ultimate quatrion rotation

    return(extract_vector_from_quaternion(r))


def print_vector(v):
    """v is a vector"""

    print("vx: " + str(v.vx) + " | vy: " + str(v.vy) + " | vz: " + str(v.vz))


def print_quaternion(q):
    """q is a quaternion"""

    print("q0: " + str(q.q0) + " | q1: " + str(q.q1) +
          " | q2: " + str(q.q2) + " | q3: " + str(q.q3))


def extract_vector_from_quaternion(q):
    """q is a quaternion"""

    q1 = q.q1
    q2 = q.q2
    q3 = q.q3

    v = Vector(q1, q2, q3)

    return(v)

def vector_to_quaternion(v): #vec2quat(r)
    x = (v.vx/2)
    y = (v.vy/2)
    z = (v.vz/2)

    l = ((x ** 2 + y ** 2 + z ** 2) ** 0.5)

    q0 = np.cos(l)  # sp
    q1 = np.sin(l) * x / l  # sp
    q2 = np.sin(l) * y / l  # sp
    q3 = np.sin(l) * z / l  # sp
    q  = Quaternion(q0, q1, q2, q3)

    return q

def angular_rate_to_quaternion_rotation(w, dt):
    """w is the vector indicating angular rate in the reference frame of the
    IMU, all coords in rad/s
    dt is the time interval during which the angular rate is valid"""

    wx = w.vx
    wy = w.vy
    wz = w.vz
    # equavelent to np.linalg.norm(w)
    l = (wx**2 + wy**2 + wz**2)**0.5 # magnitude anngle theta of rotation

    dtlo2 = dt * l / 2   #dt is first step in integration

    q0 = np.cos(dtlo2)  # sp
    q1 = np.sin(dtlo2) * wx / l  # sp - wx/l is normlizing
    q2 = np.sin(dtlo2) * wy / l  # sp
    q3 = np.sin(dtlo2) * wz / l  # sp

    r = Quaternion(q0, q1, q2, q3)

    return(r)

def accelration_to_quaternion_rotation(a, dt):
    """w is the vector indicating angular rate in the reference frame of the
    IMU, all coords in rad/s
    dt is the time interval during which the angular rate is valid"""

    ax = a.vx
    ay = a.vy
    az = a.vz

    l = (ax**2 + ay**2 + az**2)**0.5

    dtlo2 = dt * dt * l / 4

    q0 = np.cos(dtlo2)  # sp
    q1 = np.sin(dtlo2) * ax / l  # sp
    q2 = np.sin(dtlo2) * ay / l  # sp
    q3 = np.sin(dtlo2) * az / l  # sp

    r = Quaternion(q0, q1, q2, q3)

    return(r)

def angle_axis_from_unit_quaternion(q):
    """q is a unit quaternion"""

    angle = np.acos(q.q0) * 2  # sp
    sin_angle = np.sin(angle)  # sp
    axis_x = q.q1 / sin_angle
    axis_y = q.q2 / sin_angle
    axis_z = q.q3 / sin_angle

    return(angle, axis_x, axis_y, axis_z)


def compute_quaternion_norm(q):
    """q is a quaternion"""

    return((q.q0**2 + q.q1**2 + q.q2**2 + q.q3**2)**0.5)


def compute_vector_norm(v):
    """[+] v is a vector"""

    return((v.vx**2 + v.vy**2 + v.vz**2)**0.5)


def normalise_quaternion(q):
    """q is a quaternion"""

    l = compute_quaternion_norm(q)

    return(Quaternion(q.q0 / l, q.q1 / l, q.q2 / l, q.q3 / l))


def add_vectors(v, w):
    """v, w are vectors"""

    vx = v.vx
    vy = v.vy
    vz = v.vz

    wx = w.vx
    wy = w.vy
    wz = w.vz

    r = Vector(vx + wx, vy + wy, vz + wz)

    return(r)


def roll_pitch_yaw(q):
    """q is a quaternion"""

    x, y, z, w = q.q1, q.q2, q.q3, q.q0
    pitch = math.atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z)
    roll = math.atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z)
    yaw = math.asin(2 * x * y + 2 * z * w)

    return (roll, pitch, yaw)
