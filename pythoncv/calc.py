import numpy as np
import math


# Returns unit vector
def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def get_angle(mc):
    x1 = mc[0][0][0]
    y1 = mc[0][0][1]
    x2 = mc[0][1][0]
    y2 = mc[0][1][1]
    x3 = mc[0][2][0]
    y3 = mc[0][2][1]
    x4 = mc[0][3][0]
    y4 = mc[0][3][1]
    
    mx1 = (x1 + x2)/ 2.0
    my1 = (y1 + y2) / 2.0
    mx2 = (x3 + x4) / 2.0
    my2 = (y3 + y4) / 2.0
    mX = (x3 + x4) / 2.0
    mY = (y3 + y4) / 2.0
    centerX = (mx1 + mx2)/ 2.0
    centerY = (my1 + my2) / 2.0

    v1 = unit_vector([centerX-mX, centerY-mY])
    v2 = unit_vector([-1, 0])
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))

    if mY - centerY > 0:
        return 2 * math.pi - angle

    return angle