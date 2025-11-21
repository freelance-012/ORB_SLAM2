import scipy.spatial.transform as transform
from tf.transformations import quaternion_from_euler

# 角度均以弧度制表示
def YPR2R(ypr, cls="ZYX", degrees=False):
    R = transform.Rotation.from_euler(cls, ypr, degrees).as_matrix()
    return R

def Q2R(q):
    R = transform.Rotation.from_quat(q).as_matrix()
    return R

def ExpSO3(v):
    R = transform.Rotation.from_rotvec(v).as_matrix()
    return R


def R2YPR(R, cls="ZYX", degrees=False):
    ypr = transform.Rotation.from_matrix(R).as_euler(cls, degrees)
    return ypr

def R2Q(R):
    q = transform.Rotation.from_matrix(R).as_quat()
    return q

def LogSO3(R):
    v = transform.Rotation.from_matrix(R).as_rotvec()
    return v

def V2Q(v):
    q = transform.Rotation.from_rotvec(v).as_quat()
    return q

def V2YPR(v, cls="ZYX", degrees=False):
    ypr = transform.Rotation.from_rotvec(v).as_euler(cls, degrees)
    return ypr

def Q2V(q):
    v = transform.Rotation.from_quat(q).as_rotvec()
    return v

def YPR2V(ypr, cls="ZYX", degrees=False):
    v =transform.Rotation.from_euler(cls, ypr, degrees).as_rotvec()
    return v

def Q2YPR(q, cls="ZYX", degrees=False):
    ypr = transform.Rotation.from_quat(q).as_euler(cls, degrees)
    return ypr

def YPR2Q(ypr, cls="ZYX", degrees=False):
    # q = quaternion_from_euler(ypr[0], ypr[1], ypr[2], 'rzyx')  # 内旋
    # q1 = quaternion_from_euler(ypr[0], ypr[1], ypr[2], 'szyx') # 外旋
    q2 = transform.Rotation.from_euler(cls, ypr, degrees).as_quat()  # ZYX 内旋 Z(yaw)->Y(pitch)->X(roll)  zyx 外旋
    return q2

