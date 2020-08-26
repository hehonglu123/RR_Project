
import numpy as np
import math
from numpy.linalg import inv
from scipy.linalg import logm, norm, sqrtm
from pyquaternion import Quaternion
from cvxopt import matrix

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
def euler_matrix(ai, aj, ak, axes='sxyz'):
    """Return homogeneous rotation matrix from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> R = euler_matrix(1, 2, 3, 'syxz')
    >>> np.allclose(np.sum(R[0]), -1.34786452)
    True
    >>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
    >>> np.allclose(np.sum(R[0]), -0.383436184)
    True
    >>> ai, aj, ak = (4*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)
    >>> for axes in _TUPLE2AXES.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # noqa: validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = np.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M
    
def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> np.allclose(R0, R1)
    True
    >>> angles = (4*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not np.allclose(R0, R1): print(axes, "failed")

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # noqa: validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az
    
def robotParams(robot_name):
    I3 = np.eye(3)
    ex = I3[:,0]
    ey = I3[:,1]
    ez = I3[:,2]
    if robot_name=="Sawyer":
               
        h1 = ez
        h2 = ey
        h3 = ex
        h4 = -ey
        h5 = ex
        h6 = ey
        h7 = ex
        P = np.array([[0,0,0], [0.081, 0, 0.317], [0, 0.1925, 0], [0.4, 0, 0], [0, -0.1685, 0], [0.4, 0, 0], [0,0.1363,0], [0.13375, 0, 0]]).T
        q = np.zeros((7, 1))
        H = np.array([h1, h2, h3, h4, h5, h6, h7]).T
        ttype = np.zeros((1, 7))
        """ """
        n = 7
        
        dq_bounds = np.array([[100,110], [90,90], [90,90], [170,190], [120,140], [120,140], [190,235]]).T
        dq_bounds = dq_bounds*np.pi/180
        
        return ex,ey,ez,n,P,q,H,ttype,dq_bounds
    elif robot_name=="UR5":
        
        h1 = ez
        h2 = -ey
        h3 = -ey
        h4 = -ey
        h5 = -ez
        h6 = -ey
        P = np.array([[0,0,0], [0, 0, 0.089159], [-0.425, 0, 0], [-0.39225, 0, 0], [0, -0.10915, 0], [0, 0, -0.09465], [0,-0.0823,0]]).T
        q = np.zeros((6, 1))
        H = np.array([h1, h2, h3, h4, h5, h6]).T
        ttype = np.zeros((1, 6))
        """ """
        n = 6
        
        dq_bounds = np.array([[100,110], [90,90], [90,90], [170,190], [120,140], [190,235]]).T
        dq_bounds = dq_bounds*np.pi/180
        
        return ex,ey,ez,n,P,q,H,ttype,dq_bounds

def fwdkin(q,ttype,H,P,n):
    R=np.eye(3)
    p=np.zeros((3,1))
    
    for i in range(n):        
        h_i = H[0:3,i].reshape(3, 1)
        Ri = np.eye(3)
        
        if ttype[0][i] == 0: 
            #rev
            pi = P[0:3,i].reshape(3, 1)
            p = p+np.dot(R, pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R, Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1: 
            #pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R, pi)
        else: 
            #default pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R, pi)
  
    #End Effector T
    p=p+np.dot(R, P[0:3,n].reshape(3, 1))
    
    return R, p
    
# find closest rotation matrix 
# A=A*inv(sqrt(A'*A))   
def Closest_Rotation(R):
    R_n = np.dot(R, inv(sqrtm(np.dot(R.T, R))))
    
    return R_n

# ROT Rotate along an axis h by q in radius
def rot(h, q):
    h=h/norm(h)
    R = np.eye(3) + np.sin(q)*hat(h) + (1 - np.cos(q))*np.dot(hat(h), hat(h))
    return R

def hat(h):
    if (h.shape==(3,)):
        h_hat = np.array([[0, -h[2], h[1]], [h[2], 0, -h[0]], [-h[1], h[0], 0]])
    else:
        
        h_hat = np.array([[0, -h[2][0], h[1][0]], [h[2][0], 0, -h[0][0]], [-h[1][0], h[0][0], 0]])
    
    return h_hat
    
def fwdkin_alljoints(q, ttype, H, P, n):
    R=np.eye(3)
    p=np.zeros((3,1))
    RR = np.zeros((3,3,n+1))
    pp = np.zeros((3,n+1))
    
    for i in range(n):
        h_i = H[0:3,i]
       
        if ttype[0][i] == 0:
        #rev
            pi = P[0:3,i].reshape(3, 1)
            p = p+np.dot(R,pi)
            Ri = rot(h_i,q[i])
            R = np.dot(R,Ri)
            R = Closest_Rotation(R)
        elif ttype[i] == 1: 
        #pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R,pi)
        else: 
        # default pris
            pi = (P[:,i]+q[i]*h_i).reshape(3, 1)
            p = p+np.dot(R,pi)
        
        pp[:,[i]] = p
        RR[:,:,i] = R
    
    # end effector T
    p=p+np.dot(R, P[0:3,n].reshape(3, 1))
    pp[:,[n]] = p
    RR[:,:,n] = R
    
    return pp, RR

def Joint2Collision(Closest_Pt,pp):
    link_dist = []

    for i in range(5):
        link = pp[:,i+1]-pp[:,i]
        link = link/norm(link)
        pp2c = Closest_Pt - pp[:,i]
        
        link_dist.append(norm(pp2c - abs(np.dot(pp2c, link))*link))

    J2C_Joint = link_dist.index(min(link_dist)) + 1
    if(J2C_Joint==1):
        J2C_Joint=2
        
    return J2C_Joint

def check_name_abb(link_name):
    if link_name=="link_1":
        return 1
    elif link_name=="link_2":
        return 2
    elif link_name=="link_3":
        return 3
    elif link_name=="link_4":
        return 4
    elif link_name=="link_5":
        return 5
    elif link_name=="link_6":
        return 6  

def check_name_ur(link_name):
    if link_name=="shoulder_link":
        return 1
    elif link_name=="upper_arm_link":
        return 2
    elif link_name=="forearm_link":
        return 3
    elif link_name=="wrist_1_link":
        return 4
    elif link_name=="wrist_2_link":
        return 5
    elif link_name=="wrist_3_link" or link_name=="ee_link":
        return 6   
def check_name_sawyer(link_name):
    if link_name=="right_l0":
        return 1
    elif link_name=="right_l1" or link_name=="right_l1_2":
        return 2
    elif link_name=="right_l2" or link_name=="right_l2_2":
        return 3
    elif link_name=="right_l3":
        return 4
    elif link_name=="right_l4" or link_name=="right_l4_2":
        return 5
    elif link_name=="right_l5":
        return 6   
    elif link_name=="right_l6" or link_name=="right_hand":
        return 7  



def getJacobian(q,ttype,H,P,n):
    num_joints = len(q)

    P_0_i = np.zeros((3,num_joints+1))
    R_0_i = np.zeros((3,3,num_joints+1))


    P_0_i,R_0_i=fwdkin_alljoints(q,ttype,H,P,n)
    
    P_0_T = P_0_i[:,num_joints]

    J = np.zeros((6,num_joints))
    
    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i],H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))
    """ """
    
    return J

""" """
# return jacobian of the closest point on panel  
def getJacobian3(q,ttype,H,P,n, Closest_Pt, J2C_Joint):
    num_joints = len(q)

    P_0_i = np.zeros((3,num_joints+1))
    R_0_i = np.zeros((3,3,num_joints+1))


    P_0_i,R_0_i=fwdkin_alljoints(q,ttype,H,P,n)
    """  """
    
    P_0_T = Closest_Pt

    J = np.zeros((6,num_joints))
    
    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i],H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))
    
    return J

# return jacobian of the closest point on robot        
def getJacobian2(q,ttype,H,P,n,Closest_Pt,J2C_Joint):

    num_joints = len(q)

    P_0_i,R_0_i = fwdkin_alljoints(q,ttype,H,P,n)

    P_0_T = P_0_i[:,num_joints]

    J = np.zeros((6,num_joints))

    for i in range(num_joints):
        if ttype[0][i] == 0:
            J[:,i] = np.hstack((np.dot(R_0_i[:,:,i], H[:,i]), np.dot(hat(np.dot(R_0_i[:,:,i], H[:,i])), (P_0_T - P_0_i[:,i]))))

    J[:,J2C_Joint:num_joints+1] = 0                 #Modified
    link_c = P_0_i[:,J2C_Joint]-P_0_i[:,J2C_Joint-1]
    link_c = link_c/norm(link_c)
    
    P_0_tmp = P_0_i[:,J2C_Joint-1]+ abs(np.dot(Closest_Pt-P_0_i[:,J2C_Joint-1],link_c))*link_c
    
    return J,P_0_tmp

# convert a unit quaternion to angle/axis representation
def quat2axang(q):

    s = norm(q[0][1:4])
    if s >= 10*np.finfo(np.float32).eps:
        vector = q[0][1:4]/s
        theta = 2*np.arctan2(s,q[0][0])
    else:
        vector = np.array([0,0,1])
        theta = 0
    axang = np.hstack((vector,theta))
    
    return axang

def getqp_H(dq, J, vr, vp, er, ep):                 #Modified
    n = len(dq)
    H1 = np.dot(np.hstack((J,np.zeros((6,2)))).T,np.hstack((J,np.zeros((6,2)))))
    
    tmp = np.vstack((np.hstack((np.hstack((np.zeros((3,n)),vr)),np.zeros((3,1)))),np.hstack((np.hstack((np.zeros((3,n)),np.zeros((3,1)))),vp)))) 
    H2 = np.dot(tmp.T,tmp)

    H3 = -2*np.dot(np.hstack((J,np.zeros((6,2)))).T, tmp)
    H3 = (H3+H3.T)/2;
    if (n==7):
        tmp2 = np.vstack((np.array([0,0,0,0,0,0,0,np.sqrt(er),0]),np.array([0,0,0,0,0,0,0,0,np.sqrt(ep)])))
    else:
        tmp2 = np.vstack((np.array([0,0,0,0,0,0,np.sqrt(er),0]),np.array([0,0,0,0,0,0,0,np.sqrt(ep)])))
    H4 = np.dot(tmp2.T, tmp2)

    H = 2*(H1+H2+H3+H4)

    return H

def getqp_f(dq, er, ep):        #Modified
    if (len(dq)==7):
        f = -2*np.array([0,0,0,0,0,0,0,er,ep]).reshape(9, 1)
    else:
        f = -2*np.array([0,0,0,0,0,0,er,ep]).reshape(8, 1)

    
    return f

def inequality_bound(h,c,eta,epsilon,e):
    sigma = np.zeros((h.shape))
    h2 = h - eta
    sigma[np.array(h2 >= epsilon)] = -np.tan(c*np.pi/2)
    sigma[np.array(h2 >= 0) & np.array(h2 < epsilon)] = -np.tan(c*np.pi/2/epsilon*h2[np.array(h2 >= 0) & np.array(h2 < epsilon)])
    sigma[np.array(h >= 0) & np.array(h2 < 0)] = -e*h2[np.array(h >= 0) & np.array(h2 < 0)]/eta
    sigma[np.array(h < 0)] = e
    
    return sigma

# quaternion multiply
def quatmultiply(q1, q0):
    w0, x0, y0, z0 = q0[0][0], q0[0][1], q0[0][2], q0[0][3]
    w1, x1, y1, z1 = q1[0][0], q1[0][1], q1[0][2], q1[0][3]
    
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64).reshape(1, 4)


def is_pos_def(x):
    return np.all(np.linalg.eigvals(x)>0)   

def min(a,b):
    if a<b:
        return a
    else:
        return b
def normalize_w(w):
    return w/(norm(w)*5)
def normalize_x(x):
    return x/(norm(x)*5)   
def normalize_dq(q):

    q=0.5*q/(norm(q)) 

    # if (abs(q[-1]>1.)):
    #     q[-1]=q[-1]/(norm(q[-1])) 

    return q          
def normalize_dq_qp(q):

    q=0.01*q/(norm(q)) 

    # if (abs(q[-1]>0.3)):
    #     q[-1]=0.3*q[-1]/(norm(q[-1])) 

    return q 