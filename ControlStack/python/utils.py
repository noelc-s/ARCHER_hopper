# utils for main.py script
import numpy as np
import math as mt

# log operator, Log: S^3 -> s^3 -> R^3, alternatively can use manifpy
def Log(quat):

    w = quat[0]
    v = quat[1:3+1]
    
    # from Micro Lie thoery
    s = 2*v*mt.atan2(np.linalg.norm(v), w) / np.linalg.norm(v)
    return s

# exp operator, Exp: R^3 -> s^3 -> S^3, alternatively can use manifpy
def Exp(s):
    
    # from Micro Lie theory
    qw = mt.cos(np.linalg.norm(s)/2)
    qv = s * mt.sin(np.linalg.norm(s)/2) / np.linalg.norm(s)
    
    quat = [qw, qv[0], qv[1], qv[2]]
    return quat

# truncate xf down to lower dimension (need to convert quaternion)
def truncate(xf):
    # convert quaternion to euler angles
    quat  = xf[3:6+1]
    
    # remove unused incdices
    rm_idx = [7,8,9,10,17,18,19,20] # remove L,fw_pos, Ldot, fw_vel
    xf_low_dim = np.delete(xf,rm_idx)

    return xf_low_dim

# assumes normalized quaternion & converts to Euler angles in 3-2-1 sequence
# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def toEulerAngles(quat):
    
    # unpack quaternion 
    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    # roll x-axis
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    r = mt.atan2(sinr_cosp, cosr_cosp)

    # pitch y-axis
    sinp = mt.sqrt(1 + 2 * (qw * qy - qx * qz))
    cosp = mt.sqrt(1 - 2 * (qw * qy - qx * qz))
    p = 2 * mt.atan2(sinp, cosp) - mt.pi / 2

    # yaw z-axis
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    y = mt.atan2(siny_cosp, cosy_cosp)

    return r,p,y

# euler angles to quaternion
def toQuaternion(r,p,y):

    cr = mt.cos(r * 0.5)
    sr = mt.sin(r * 0.5)
    cp = mt.cos(p * 0.5)
    sp = mt.sin(p * 0.5)
    cy = mt.cos(y * 0.5)
    sy = mt.sin(y * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz


# truncate xf down to lower dimension (need to convert quaternion)
# quat  = xf[3:6+1]
# rm_idx = [7,8,9,10,17,18,19,20] # remove L,fw_pos, Ldot, fw_vel
# xf_low_dim = np.delete(xf,rm_idx)
# print("xf low dim: "); print(xf_low_dim)
