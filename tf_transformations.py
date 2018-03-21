#! /usr/bin/env python

#title		 :tf_transformations.py
#description     :This is for transformation between Quaternion and Euler. 
#author          :Salah Eddine Ghamri
#date            :15-03-2018
#version         :0.2
#usage           :python pyscript.py
#notes           :The position of landmarks are stored here (change if needed).
#python_version  :2.6.7  
#==============================================================================
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R

def mat2quat(M):
    ''' Calculate quaternion corresponding to given rotation matrix

    Parameters
    ----------
    M : array-like
      3x3 rotation matrix

    Returns
    -------
    q : (4,) array
      closest quaternion to input matrix, having positive q[0]

    Notes
    -----
    Method claimed to be robust to numerical errors in M

    Constructs quaternion by calculating maximum eigenvector for matrix
    K (constructed from input `M`).  Although this is not tested, a
    maximum eigenvalue of 1 corresponds to a valid rotation.

    A quaternion q*-1 corresponds to the same rotation as q; thus the
    sign of the reconstructed quaternion is arbitrary, and we return
    quaternions with positive w (q[0]).

    References
    ----------
    * http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    * Bar-Itzhack, Itzhack Y. (2000), "New method for extracting the
      quaternion from a rotation matrix", AIAA Journal of Guidance,
      Control and Dynamics 23(6):1085-1087 (Engineering Note), ISSN
      0731-5090

    Examples
    --------
    >>> import numpy as np
    >>> q = mat2quat(np.eye(3)) # Identity rotation
    >>> np.allclose(q, [1, 0, 0, 0])
    True
    >>> q = mat2quat(np.diag([1, -1, -1]))
    >>> np.allclose(q, [0, 1, 0, 0]) # 180 degree rotn around axis 0
    True

    '''
    # Qyx refers to the contribution of the y input vector component to
    # the x output vector component.  Qyx is therefore the same as
    # M[0,1].  The notation is from the Wikipedia article.
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    # Fill only lower half of symmetric matrix
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    # Use Hermitian eigenvectors, values for speed
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector, reorder to w,x,y,z quaternion
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    # Prefer quaternion with positive w
    # (q * -1 corresponds to same rotation as q)
    if q[0] < 0:
        q *= -1
    return q



marker_quaternion = ((-4.309, -1.589, 0.31),(0.0, 0.0, 0.0, 1))
pi = np.pi
def rot_z(q):
	rot_z = np.array(((np.cos(q),(-np.sin(q)),0),
			  (np.sin(q),np.cos(q),0),
			  (0,0,1)))
	return rot_z
def rot_x(q):
	rot_x = np.array(((1,0,0),
        	         (0,np.cos(q),-np.sin(q)),
               		 (0,np.sin(q),np.cos(q))))
	return rot_x
def rot_y(q):
	rot_y = np.array(((np.cos(q), 0, -np.sin(q)),
			  (0,1,0),
			  (np.sin(q),0,np.cos(q))))
	return rot_y

# from euler to quaternion
roll = 0
pitch = 0
yaw = np.pi/2 

quaternion = quaternion_from_euler(roll, pitch, yaw)
# type(pose) = geometry_msgs.msg.Pose
orientation_x = quaternion[0]
orientation_y = quaternion[1]
orientation_z = quaternion[2]
orientation_w = quaternion[3]

#print (orientation_x, orientation_y, orientation_z, orientation_w)


orientation_x = 0.0 
orientation_y = 0.0
orientation_z = 0.0
orientation_w = 0.1



# from quaternion to euler 
quaternion = (
    orientation_x,
    orientation_y,
    orientation_z,
    orientation_w)
euler = euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

theta = (roll,pitch,yaw)
a = eulerAnglesToRotationMatrix(theta)
#print(roll,pitch,yaw),
f = np.dot(rot_x(0),rot_y(pi/2))
f = np.dot(f, rot_z(0))
f = np.dot(f, a)
q = mat2quat(f)
print ("x: %r \ny: %r \nz: %r \nw: %r" % (q[1],q[2],q[3],q[0])),
