# %% Imports

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation
from scipy.linalg import expm


# %%

def complementary_filter_update(initial_rotation, angular_velocity, linear_acceleration, dt):
    """
    Implements a complementary filter update

    :param initial_rotation: rotation_estimate at start of update
    :param angular_velocity: angular velocity vector at start of interval in radians per second
    :param linear_acceleration: linear acceleration vector at end of interval in meters per second squared
    :param dt: duration of interval in seconds
    :return: final_rotation - rotation estimate after update
    """

    # TODO Your code here - replace the return value with one you compute

    '''
    >>> Construct quaternion multiply together to obtain estimate
    >>> Using measured acceleration vector compute erroor meeasure by looking at magnnitude of accelration vectoor,
    >>> compare to 1 or 9.8
    >>> Compute gain alpha based on output. If nonn-zero, compute correction matrix
    >>> Apply rotation corection using gain alpha
    
    '''

    # Accelaration due to gravity and alpha slope
    g = 9.81
    gain_slope = -10

    # Use linear ODE model to compute time position rotation matrix and form to quaternnion
    curr_quaternion = Rotation.from_matrix(expm(skew(angular_velocity) * dt)).as_quat()
    arr = curr_quaternion[:-1]

    # real = np.cos(dt / 2)
    # imag = np.sin(dt / 2) * angular_velocity
    # curr_quaternion = np.append(real, imag)

    # Do quaternion multiplication to update rotation matrix
    prev_quaternion = initial_rotation.as_quat()[::-1]
    real_estimate = prev_quaternion[0] * curr_quaternion[0] - prev_quaternion[1:] @ curr_quaternion[1:]
    imag_estimate = prev_quaternion[0] * curr_quaternion[1:] + curr_quaternion[0] * prev_quaternion[1:] \
                    + skew(prev_quaternion[1:]) @ curr_quaternion[1:]
    quat_estimate = np.append(real_estimate, imag_estimate)

    # Computer error magnitude of acceleration vector
    error_measured = np.abs(np.linalg.norm(linear_acceleration) - g)

    # Compute g_prime
    rot_estimate = Rotation.from_quat([quat_estimate[1], quat_estimate[2], quat_estimate[3],
                                       quat_estimate[0]]).as_matrix()
    g_prime = rot_estimate @ linear_acceleration

    # Construct quaternion correction
    imag_correct = np.array([np.sqrt((g_prime[2] + 1) / 2), g_prime[1] / np.sqrt(2 * (g_prime[2] + 1)),
                             -g_prime[0] / np.sqrt(2 * (g_prime[2] + 1))])

    quat_correct = np.append(0, imag_correct)

    # Compute alpha



     # Do quaternion multiplication to update rotation matrix

    # Update rotation using measured angular velocity (How to factor initial condiiton??)
    # omega_hat = np.array([[0, -angular_velocity[2], angular_velocity[1]],
    #                       [angular_velocity[2], 0, -angular_velocity[0]],
    #                       [-angular_velocity[1], angular_velocity[0], 0]])
    #
    # r_estimate = initial_rotation.as_matrix() * np.exp(omega_hat * dt)



    # Convert current rotation matrix

    return Rotation.identity()

# rot_estimate = (quat_estimate[0] ** 2 - quat_estimate[1:] @ quat_estimate[1:]) * Rotation.identity().as_matrix() + \
#                2 * quat_estimate[0] * skew(quat_estimate[1:]) +  2 * quat_estimate[1:][:, None] @ quat_estimate[1:][None, :]

def skew(v):
    """
        This function computes the skew symmetric representation of a (3, ) vector

        Parameters:
            v,    (3, ) numpy array represennting i,j,k counterpart of quaternion

        Return: 3x3 numpy array

    """

    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])
