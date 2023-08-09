#!/home/ape/miniconda3/envs/flask/bin/python

import rospy
import numpy as np
import math

import time
from scipy.spatial import cKDTree

# structure of the nearest neighbor 
class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []

class ICP:
    def __init__(self):
        # max iterations
        self.max_iter = rospy.get_param('/icp/max_iter',50)
        # distance threshold for filter the matching points
        self.dis_th = rospy.get_param('/icp/dis_th',1.5)
        # tolerance to stop icp
        self.tolerance = rospy.get_param('/icp/tolerance',6)
        # min match
        self.min_match = rospy.get_param('/icp/min_match',2)
    
    # ICP process function
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def process(self,tar_pc,src_pc):
        transform_acc = np.identity(3)
        iter_cnt = 0
        dError = (float)("inf")
        preError = (float)("inf")

        for _ in range(self.max_iter):
            neigh = self.findNearest_KD_Tree(np.array(src_pc[0:2, :]), np.array(tar_pc[0:2, :]))
            print(neigh.src_indices.shape)

            T = self.getTransform(src_pc[0:2, neigh.src_indices], tar_pc[0:2, neigh.tar_indices])

            tar_pc = np.dot(self.R, tar_pc[0:2])+T[:, np.newaxis]

            transform_acc = self.update_trans(transform_acc, self.R, T)

            dError = abs(preError - sum(neigh.distances))
            preError = sum(neigh.distances)

            if dError < self.tolerance:
                print("error",preError)
                break

            iter_cnt += 1
            pass

        print("total_iter: ", iter_cnt)
        return transform_acc

    def update_trans(self, Hin, R, T):
        H = np.zeros((3, 3))

        H[0, 0] = R[0, 0]
        H[1, 0] = R[1, 0]
        H[0, 1] = R[0, 1]
        H[1, 1] = R[1, 1]
        H[2, 2] = 1.0

        H[0, 2] = T[0]
        H[1, 2] = T[1]

        return np.dot(H,Hin)

    # find the nearest points & filter
    # return: neighbors of src and tar
    def findNearest_KD_Tree(self,src,tar):
        neigh = NeighBor()

        mytree = cKDTree(src.T)
        distances, src_indices = mytree.query(tar.T)
        
        neigh.distances = distances[distances<self.dis_th]
        neigh.src_indices = src_indices[distances<self.dis_th]
        tar_index = np.linspace(0, tar.shape[1], tar.shape[1], endpoint=False,dtype=int)
        neigh.tar_indices = tar_index[distances<self.dis_th]
        
        return neigh


    def findNearest_Numpy(self,src,tar):
        neigh = NeighBor()

        d = np.linalg.norm(np.repeat(tar, src.shape[1], axis=1) - np.tile(src, (1, tar.shape[1])), axis=0)
        src_indices = np.argmin(d.reshape(tar.shape[1], src.shape[1]), axis=1)

        distances = np.min(d.reshape(tar.shape[1], src.shape[1]), axis=1)
        neigh.distances = distances[distances<self.dis_th]
        neigh.src_indices = src_indices[distances<self.dis_th]
        tar_index = np.linspace(0, tar.shape[1], tar.shape[1], endpoint=False,dtype=int)
        neigh.tar_indices = tar_index[distances<self.dis_th]
        return neigh


    # Waiting for Implementation 
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def getTransform(self,src,tar):
        T = np.identity(3)
        src = np.array(src)
        tar = np.array(tar)

        src_p = np.mean(src, axis=1)
        tar_p = np.mean(tar, axis=1)

        src_new = src - src_p[:, np.newaxis]
        tar_new = tar - tar_p[:, np.newaxis]

        W = np.dot(tar_new, src_new.T)

        U, _, VT = np.linalg.svd(W)

        self.R = np.dot(VT.T, U.T)
        T = src_p - np.dot(self.R, tar_p)

        return T

    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)
