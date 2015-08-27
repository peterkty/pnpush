import os
import json
import sys
import pdb
import numpy as np
import tf
import rospy
import tf.transformations as tfm
import time
from numpy import linalg as la
import traceback
from roshelper import lookupTransform

listener = None

def xyzrpy_from_xyzquat(pose):
    return pose[0:3] + list(tfm.euler_from_quaternion(pose[3:7])) # x,y,z,qx,qy,qz,qw

def matrix_from_xyzquat(translate, quaternion):
    return np.dot(tfm.compose_matrix(translate=translate) , 
                   tfm.quaternion_matrix(quaternion)).tolist()

def transformBack(tf_xyzquat, pose):
    T_mat = tfm.concatenate_matrices( tfm.translation_matrix(tf_xyzquat[0:3]), tfm.quaternion_matrix(tf_xyzquat[3:7]))
    pose_mat = tfm.concatenate_matrices( tfm.translation_matrix(pose[0:3]),  tfm.quaternion_matrix(pose[3:7]) )
    new_pose_mat = np.dot(pose_mat, tfm.inverse_matrix(T_mat))
    return tfm.translation_from_matrix(new_pose_mat).tolist() + tfm.quaternion_from_matrix(new_pose_mat).tolist()


# something useful for building primitives
import geometry_msgs.msg

def pause():
    raw_input('Press any key to continue')

    
def getObjCOM(objPose, objId):
    #gives you the center of mass of the object    
    # object frame is attached at com
    objPosition = objPose[0:3]
    return objPosition



def matrix_from_xyzquat(translate, quaternion):
    return np.dot(tfm.compose_matrix(translate=translate) , 
                   tfm.quaternion_matrix(quaternion)).tolist()


def quat_from_matrix(rot_matrix):
    return (tfm.quaternion_from_matrix(rot_matrix))


def rotmatY(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[np.cos(theta_rad), 0, np.sin(theta_rad)],
    [0, 1, 0],[-np.sin(theta_rad), 0, np.cos(theta_rad)]]))


def rotmatX(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[1, 0, 0],
    [0, np.cos(theta_rad), -np.sin(theta_rad)],[0, np.sin(theta_rad), np.cos(theta_rad)]]))

def rotmatZ(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[np.cos(theta_rad), -np.sin(theta_rad), 0],[np.sin(theta_rad), np.cos(theta_rad), 0], [0,0,1]]))
    
class Timer(object):
    #### Timer to time a piece of code
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print '\t[%s]' % self.name,
        print 'Elapsed: %s' % (time.time() - self.tstart)

class shortfloat(float):
    def __repr__(self):
        return "%0.3f" % self
