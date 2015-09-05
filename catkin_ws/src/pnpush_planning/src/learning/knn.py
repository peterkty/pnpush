#!/usr/bin/env python

# Peter KT Yu, Aug 2015
# Use knn to predict motion of the object given push location and velocity

import numpy as np
import matplotlib.pyplot as plt

from sklearn.datasets import fetch_olivetti_faces
from sklearn.utils.validation import check_random_state

from sklearn.ensemble import ExtraTreesRegressor
from sklearn.neighbors import KNeighborsRegressor
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import RidgeCV

import sys, json
from ik.helper import *

def main(argv):
    
    # prepare data
    
    inputfile= "%s/data_training.json" % argv[1]
    # the data is a 2-d array with these labels for columns
    labels = ['tip_x', 'tip_y', 'tip_vx', 'tip_vy', 'forcex', 'forcey', 'object_pose_vx', 'object_pose_vy', 'object_pose_vtheta']
    
    with open(inputfile) as data_file:    
        data = json.load(data_file)
    
    data = np.random.permutation(data).tolist()
    
    k = int(argv[2])
    
    # cross validation
    n_cross = 5
    n_data = len(data)
    n_perseg = n_data/n_cross
    error_xy = 0
    error_angle = 0
    for i in range(n_cross):
        test_seg_begin = i * n_perseg
        test_seg_end = ((i+1) * n_perseg) if (i < n_cross - 1) else (n_data)
        n_test = test_seg_end - test_seg_begin
        
        X_train = np.array(data[0:test_seg_begin] + data[test_seg_end:n_data])[:,0:4].tolist()
        y_train = np.array(data[0:test_seg_begin] + data[test_seg_end:n_data])[:,6:9].tolist()
        X_test = np.array(data[test_seg_begin:test_seg_end])[:,0:4].tolist()
        y_test = np.array(data[test_seg_begin:test_seg_end])[:,6:9].tolist()
        
        estimator = KNeighborsRegressor(n_neighbors=k)
        estimator.fit(X_train, y_train)
        y_test_predict = estimator.predict(X_test)
        
        error_xy += norm((np.array(y_test_predict) - np.array(y_test))[:,0:2].flatten(1)) / n_test
        error_angle += norm((np.array(y_test_predict) - np.array(y_test))[:,2].flatten(1)) / n_test
            
    error_xy /= n_cross
    error_angle /= n_cross

    std_xy = (norm(np.array(data)[:,6:8].flatten(1))**2 / n_data)**0.5
    std_angle = (norm(np.array(data)[:,8].flatten(1))**2 / n_data)**0.5
    
    #print(data[:4])
    #plt.plot(np.array(data)[:,6].tolist())
    #plt.show()

    print 'error_xy', error_xy, 'error_angle', error_angle
    print 'var_xy', std_xy, 'std_angle', std_angle
    print 'error_xy_percent', error_xy/std_xy, 'error_angle_percent', error_angle/std_angle

if __name__=='__main__':
    main(sys.argv)



