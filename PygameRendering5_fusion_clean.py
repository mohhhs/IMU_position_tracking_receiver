import sys
import pygame
from operator import itemgetter
import Quaternions as qt
import numpy as np
import plotly.graph_objects as go
from scipy.signal import butter,filtfilt


# 02-28-20 need to update the screen keep zooming?

class Point3D:
    """A class used for describing a point in 3D."""

    def __init__(self, x=0, y=0, z=0):
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.v = qt.Vector(self.x, self.y, self.z)

    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, self.z)

    def rotateQ(self, q):
        """Apply rotation described by quaternion q to this 3D point"""
        v_rotated = qt.apply_rotation_on_vector(q, self.v)
        return Point3D(v_rotated.vx, v_rotated.vy, v_rotated.vz)

    def posupdate(self,p0,p1,p2):
        # apply postion from accelroameter on point
        x = self.x+float(p0)
        y = self.y+float(p1)
        z = self.z+float(p2)
        return Point3D(x,y,z)


class RenderGyroIntegration:
    """A class for rendering gyro integration as a 3D cube display."""


    def __init__(self, GYR_Integration_Instance, win_width=640, win_height=480):
        self.GYR_Integration_Instance = GYR_Integration_Instance
        '''[3]
        pygame.init()

        self.screen = pygame.display.set_mode((win_width, win_height))
        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption("Rendering of 3D cube")
        [3]'''
        self.clock = pygame.time.Clock()

        self.vertices = [   # 8 points dim of cube
            Point3D(-1, 1, -1), #vertics 0
            Point3D(1, 1, -1),  #vertics 1
            Point3D(1, -1, -1),
            Point3D(-1, -1, -1),
            Point3D(-1, 1, 1),
            Point3D(1, 1, 1),
            Point3D(1, -1, 1),
            Point3D(-1, -1, 1)  #vertics 7
        ]
        # Define the vertices that compose each of the 6 faces.
        # each fase consistent of 4 vertices
        self.faces = [(0, 1, 2, 3), (1, 5, 6, 2), (5, 4, 7, 6),
                      (4, 0, 3, 7), (0, 4, 5, 1), (3, 2, 6, 7)]

        # Define colors for each face
        self.colors = [(255, 0, 255), (255, 0, 0), (0, 255, 0),
                       (0, 0, 255), (0, 255, 255), (255, 255, 0)]

        self.angle = 0

    def run(self,norm,dtt, v_off , p_off): #maybe add norm directly will be different
        """ Main Loop: run until window gets closed."""
        velocity = [0] * 3
        position = [0] * 3
        my_array = np.loadtxt('imu_data-3-GyroRefSet.csv', delimiter=",", skiprows=1)
        count = 0
        gyro_range = 250

        cuttoff = 5
        order = 4
        fs = 38

        # first numbers is data set imu_data-3-GyroRefSet.csv (you could just do if else for first run - later)
        sens_bias = [0, -0.02911, 0.00821, 0.04021, 0.00871, -0.00091, -0.00171]  # A_x-y-z G_x-y-z
        KI_a = [[], [], []]
        acc_angs = [[], [], []]
        gyr_angs = [[], [], []]
        gyr_q = [[], [], [], []]
        fus_q = [[], [], [], []]
        deg = 180 / np.pi

        #gravity compensation params
        gravity = qt.Vector(0, 0, -1)  # in Gs

        #UKF parameters
        P = 0.00001 * np.identity(3)  # convariance 3*3 symatric
        Q = 0.00001 * np.identity(3)  # state noise convariance 3*3 symatric
        R = 0.0001 * np.identity(3)  # convariance of sensor noise
        q0 = qt.Quaternion(1, 0, 0, 0)  # initial mean
        qt_1 = qt.Quaternion(1, 0, 0, 0)  # new mean predicted
        g = qt.Quaternion(0, 0, 0, 0.0000000000001)  # quaternion for rotation by pi on z (I belive this is to remove gravity)

        ''' filtering not working with angles, not sure why. filter after angle conversion
        aa_off = [0.034, 0.109, 1.045]  # x,y,z accelration offset
        aa = [[], [], []]
        # loop to get filtered accelorameter
        for af in my_array:
            aa[0].append((af[0 + 1] - aa_off[0]))  # a.x
            aa[1].append((af[1 + 1] - aa_off[1]))  # a.y
            aa[2].append((af[2 + 1] - aa_off[2]))  # a.z

        aaxf = butter_lowpass_filter(aa, cuttoff, fs, order)  # filtered aa.x
        '''

        for line in my_array:

            if count == 1000: # to keep it short so it doen't go to 1500
                break
            print(count)
            '''[2]
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            
            self.clock.tick(39) # makes sure we don't run over 40 frames per second
            self.screen.fill((0, 32, 0))
            [2]'''

            # It will hold transformed vertices.
            t = []

            # perform one gyro integration: read, update quaternion
            # self.GYR_Integration_Instance.perform_one_iteration()  #gets V , and e^((thita/2)*n)
            #q = self.GYR_Integration_Instance    #.o   #mag of qDelta in ??
            am = qt.Vector(line[2] - sens_bias[2], line[1] - sens_bias[1], line[3] - sens_bias[3])  #make accelration vector - switching 1 , 2 to match gyro
            wm = qt.Vector(line[4] - sens_bias[4], line[5] - sens_bias[5], line[6] - sens_bias[6])  # make gyro vector instance
            qAlfa = qt.accelration_to_quaternion_rotation(am, dtt)
            #qDelta is euqevelent to qu = vec2quat(ut) in ukf.py line 70ish
            qDelta = qt.angular_rate_to_quaternion_rotation(wm, dtt)

            '''acc gravity compensation(migh or might not need it)'''
            norm = qt.quaternion_product(norm, qDelta)
            # [1] quaternion from gyroscope
            q = qt.normalise_quaternion(norm)  # quaternion describes cureent rotation

            am_val = am.array()[0] # uncompensated
            acc_rot = qt.apply_rotation_on_vector(q, am)  # rotate gravity to earth coordinations
            acc_dyn = qt.add_vectors(acc_rot, gravity)  # compensated gravity
            acc_val = acc_dyn.array()[0] # compansated

            '''ukf code starts with gaussian_update'''

            next_q, next_cov, sigmas, error = gussian_update(qt_1, qDelta, P, Q)
            z, z_mean = sigma_update(sigmas, g, R)
            z = np.matrix(z)  # convert from array to matrix (6*3)
            z_mean = np.matrix(z_mean)  # convert from array to matrix (1,3)

            pzz = calcpzz(z, z_mean)
            pvv = pzz + R
            pxz = calcpxz(error, z, z_mean)

            K = np.dot(pxz, np.linalg.inv(pvv))

            I = np.transpose(acc_val - z_mean)  # matrix (3*1)
            KI_array = np.transpose(K * I).A1  # converting to array ([1,2,3]) (3*1)
            KI_vector = qt.Vector(KI_array[0], KI_array[1], KI_array[2])
            KI = qt.vector_to_quaternion(KI_vector)  # quaternion
            # [2] quaternon of gyro + accelorameter
            qt_1 = qt.quaternion_product(KI, next_q)  # quaternion fusion of ukf
            P = next_cov - np.dot(np.dot(K, pvv), np.transpose(K))  #final covariance (3*3) array ([[1,2]])

            # orignal q from gyro
            gyr_q[0].append(float("{0:.4f}".format(q.q0))*50) # *50 just so I see
            gyr_q[1].append(float("{0:.4f}".format(q.q1))*deg)
            gyr_q[2].append(float("{0:.4f}".format(q.q2))*deg)
            gyr_q[3].append(float("{0:.4f}".format(q.q3))*deg)

            # fused q from acc and gyro
            fus_q[0].append(float("{0:.4f}".format(qt_1.q0))*50) # *50 just so I see
            fus_q[1].append(float("{0:.4f}".format(qt_1.q1))*deg)
            fus_q[2].append(float("{0:.4f}".format(qt_1.q2))*deg)
            fus_q[3].append(float("{0:.4f}".format(qt_1.q3))*deg)

            acc_deg = acc_angle(am)
            gyr_deg = qt.roll_pitch_yaw(q)

            # for i in range(1):
            #     acc_angs[i].append(acc_deg[i])
            #     gyr_angs[i].append(gyr_deg[i]*deg) # convert from rad to degree

            for i in range(3):
                acc_angs[i].append(acc_val[i])
                gyr_angs[i].append(z_mean.A1[i]) # convert from rad to degree
                KI_a[i].append(KI_array[i])

            #it will hold x y z postion
            pos = []
            for i in range(1):
                # integrate accelaration to velocity for dtt rate
                velocity[i] = velocity[i] + (line[i+1] * dtt) + v_off
                # integrate velocity to postion for dtt rate
                position[i] = position[i] + (velocity[i] * dtt) + p_off
                pos.append(position[i])



            # # integrate accelaration to velocity for dtt rate
            # velocity[0] = velocity[0]+(line[1] * dtt) + v_off
            # velocity[1] = velocity[1]+(line[2] * dtt) + v_off
            # velocity[2] = velocity[2]+(line[3] * dtt) + v_off
            #
            # # integrate velocity to postion for dtt rate
            # position[0] = position[0]+(velocity[0] * dtt) + p_off
            # position[1] = position[1] + (velocity[1] * dtt) + p_off
            # position[2] = position[2] + (velocity[2] * dtt) + p_off


            '''[1]comment out for testing angles
            for v in self.vertices:
                #adding the postion from acclrorameter
                v_pos = v.posupdate(pos[0],pos[1],pos[2])

                # rotate point according to integrated gyro
                # here where you want to add postion
                r = v_pos.rotateQ(q)
                # Transform the point from 3D to 2D
                p = r.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
                # Put the point in the list of transformed 6 vertices
                t.append(p)

            # Calculate the average Z values of each face.
            avg_z = []
            i = 0
            # each face has 4 points f[0]...[3]
            for f in self.faces:
                z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z) / 4.0
                avg_z.append([i, z])
                i = i + 1

            # Draw the faces using the Painter's algorithm:
            # Distant faces are drawn before the closer ones.
            for tmp in sorted(avg_z, key=itemgetter(1), reverse=True):
                face_index = tmp[0]
                f = self.faces[face_index]
                pointlist = [(t[f[0]].x, t[f[0]].y), (t[f[1]].x, t[f[1]].y),
                             (t[f[1]].x, t[f[1]].y), (t[f[2]].x, t[f[2]].y),
                             (t[f[2]].x, t[f[2]].y), (t[f[3]].x, t[f[3]].y),
                             (t[f[3]].x, t[f[3]].y), (t[f[0]].x, t[f[0]].y)]
                pygame.draw.polygon(self.screen, self.colors[face_index], pointlist)

            self.angle += 1

            pygame.display.flip() # this code had to put it in full screen for better speeds
            [1]'''
            count += 1

        # acc_angsf = butter_lowpass_filter(acc_angs, cuttoff, fs, order)  # filtered aa.x

        fig = go.Figure()
        fig.add_trace(go.Scatter(
            y=gyr_q[3],
            line=dict(shape='spline'),
            name='gyro_q3'
        ))


        fig.add_trace(go.Scatter(
            y=fus_q[3],
            line=dict(shape='spline'),
            name='UKF_q.q3'
        ))



        fig.show()


def acc_angle(acc):
    deg = 180/np.pi
    pitch_x = np.arctan2(-acc.vx, np.sqrt(np.power(acc.vy, 2)+np.power(acc.vz, 2))) * deg
    roll_y  = np.arctan2( acc.vy, np.sqrt(np.power(acc.vx, 2)+np.power(acc.vz, 2))) * deg
    yaw_z   = np.arctan2( acc.vz, np.sqrt(np.power(acc.vx, 2)+np.power(acc.vz, 2))) * deg #expermintal
    return pitch_x, roll_y, yaw_z

def butter_lowpass_filter(data, cutoff,fs, order ):
    normal_cutoff = cutoff / (0.5*fs) # denom is nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)  #this is the actual filtered data , data is an array of numbers
    return y

# note in my dataset gyro_x correlates with acc_y
'''[+]
guassian function creats sigma points and predict mean and covarience. 
takes:
qt: new mean, predicted value, quaternion
ut: quaternion (dt*gyro_vector)  
P : Covariaence 3*3 symatric
Q : state noise covariance 3*3 symatric
returns:
qt_n : predicted mean 
cov_n: covarience of mean 
sigma_points
error
'''
# test this
def gussian_update(qt_1,ut,P,Q):

    # tmp = np.matrix(np.zeros(4, 6)) # 6 sigma points holder
    dim = 6 # we are working with 6 signals (sima,motion,error)
    sigmas = [0] * dim  #list to hold 6 sigma objects
    motion_sig = [0] * dim  # holder
    error = [0] * dim  #holder
    # Cholesky decomposition for sigma points
    L = np.linalg.cholesky(P+Q)
    n,m = np.shape(P)
    left_vec = L * np.sqrt(2 * n)
    right_vec = -L * np.sqrt(2 * n)
    new_vec = np.hstack((left_vec, right_vec))
    nr, nc = np.shape(new_vec)

    v = np.matrix(np.zeros([3,6]))
    for i in range(0,nc):  #loop columns
        n_vec = qt.Vector(new_vec[0,i],new_vec[1,i],new_vec[2,i])  # converting to vector object
        temp = qt.vector_to_quaternion(n_vec)
        # work on creating a list of objects for sigma or just make six sigmas quaternion
        tmp_sg = qt.quaternion_product(temp,qt_1)  # this conpines temp rotation with qt_1 rotation
        sigmas[i] = qt.Sigma(tmp_sg.q0, tmp_sg.q1, tmp_sg.q2, tmp_sg.q3)  # 6 sigmas
        # tmp[:,i] = np.transpose(qt.quaternion_product(temp,qt))
    qt.Sigma.sig_num = 0 # after making 6 sigmas rezero the counter
    for i in range(0, 6):
        tmp_ms = qt.quaternion_product(sigmas[i], ut)
        motion_sig[i] = qt.Quaternion(tmp_ms.q0, tmp_ms.q1, tmp_ms.q2, tmp_ms.q3)
    next_qt, error = qt.quaternion_average(motion_sig, qt_1) # return quaterion(qt new mean), 6 error vectors

    next_cov = np.zeros([3, 3]) # 3*3 because it is vector
    for i in range(0,len(error)):
        ea = error[i].array() #converting error vector to numpy array
        temp_cov = np.transpose(ea)*ea
        # it doesn't like += for some
        next_cov = next_cov + temp_cov  # add 6 times all (3*3) arrays
    next_cov = next_cov/12  #new covarience

    # return quaternion, (3*3) array, 6 quaternion sigma, 6 error vectors
    return next_qt, next_cov, sigmas, error

'''[+]
Rotate each sigma by 180 about z, and calculate covarience 
I belive this to rotate sigmas to earth frame to subtract gravity
sigmas: 6 quaternion sigma point
g: quaternion rotate about z by pi (180)
R: covariance of sensor noise
z: I belive is some covariance, array (6*3)
z_mean: averge array (1*3)
'''
def sigma_update(sigmas, g, R):
    dim = len(sigmas)
    new_sigmas = [0] * dim  # hold new roated sigmas by pi about z axis
    z = np.zeros([dim,4])  # holds for evantually vector of new_simgas
    for i in range(dim):
        # simas_T * g * simas (rotating g vector (0,0,1) by sigmas) I belive this adds gravity to sigma
        # since I already got rid of gravity so this does not apply to my code
        new_sigmas[i] = qt.quaternion_product(qt.quaternion_product(qt.inverse_quaternion(sigmas[i]), g), sigmas[i])
        z[i] = new_sigmas[i].array()

    z = z[:, 1:]  # taking only vector part
    z_mean = np.mean(z, 0)  # avarage of each column

    return z, z_mean

'''[+ unmodified]
calculate covariance z z
z: matrix (6*3)
z_mean: matrix (1*3)
'''
def calcpzz(z, z_mean):
    temp = np.matrix(z - z_mean)  # subtract the mean quaternion victor from each quaternion victor (6*3)
    pzz = np.zeros([np.shape(z)[1], np.shape(z)[1]])  # (3*3)
    for i in range(0, np.shape(temp)[0]):  # range(6)
        pzz_temp = np.transpose(temp[i]) * temp[i]
        pzz += pzz_temp

    return pzz / 12.0

'''[+]
calculate covarience x z
error: is covarience of predicted mean discribed in 6 vectors
z: matrix (6*3)
z_mean: matrix (1*3)
'''
def calcpxz(error, z, z_mean):
    temp = np.matrix(z - z_mean)
    pxz = np.zeros([np.shape(z)[1], np.shape(z)[1]])  # (3*3)
    for i in range(len(error)):
        error_v = error[i].array()
        pxz_temp = np.transpose(error_v)*temp[i]
        pxz = pxz + pxz_temp

    return pxz/12.0


dt= .025
v_offset = 0.0033  # thats caused by drift from imu
p_offset = 0.0042  # same as above but for postion
norm = qt.normalise_quaternion(qt.Quaternion(1, 0.0001, 0.0001, 0.0001))
RenderGyroIntegration_instance = RenderGyroIntegration(norm)
RenderGyroIntegration_instance.run(norm,dt, v_offset , p_offset)

