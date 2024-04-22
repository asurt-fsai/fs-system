import numpy as np

class Cone():

    def __init__(self) -> None:
        
        self.position = None
        self.type = None
        self.id = None
        self.view_ids = None
        self.kps = None

class View():

    def __init__(self) -> None:
        
        self.view_id = None
        self.absolute_pose = None

def triangulate_DLT(proj_mats, pts, norm=True, imsize=None):

    N = pts.shape[1] #num views

    #pre-conditioning
    if imsize is not None:
        T = np.array([[2/imsize[0], 0, -1],
                [0, 2/imsize[1], -1],
                [0, 0, 1]]) #pre-conditioning matrix T

        for i in range(N):
            proj_mats[:,:,i] = np.dot(T, proj_mats[:,:,i])
            pts[:, i] = np.dot(T[0:2, 0:2], pts[:, i]) + T[0:2, 2]


    #forming system of equations
    A = np.zeros((2*N, 4)) #pre-allocating memory
    for i in range(N):
        A[2*i,:] = (pts[0,i] * proj_mats[2,:,i]) - proj_mats[0,:,i] 
        A[(2*i)+1,:] = (pts[1,i] * proj_mats[2,:,i]) - proj_mats[1,:,i]

    #normalize the rows of A
    if norm:
        row_norms = np.linalg.norm(A, axis=1, keepdims=True) #Compute the L2 norm of each row
                
        A = A / row_norms #Divide each row by its norm

    #Solve for X_est using SVD
    _, _, Vh = np.linalg.svd(A, full_matrices=False)
    X_est = Vh[-1,:]
    
    #checking for correct orientation
    s = np.dot(proj_mats[2,:,:].T, X_est)
    
    if np.any(s < 0):
        X_est = -X_est

    X_est = X_est[0:3] / X_est[3] #convert from Homogeneous to Euclidean
    
    return X_est