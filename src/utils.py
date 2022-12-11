import numpy as np

def twistHat(twist):
    tHat = np.zeros([4,4])
    tHat[0,1] = -twist[5]
    tHat[0,2] = twist[4]
    tHat[1,0] = twist[5]
    tHat[1,2] = -twist[3]
    tHat[2,0] = -twist[4]
    tHat[2,1] = twist[3]
    tHat[0,3] = twist[0]
    tHat[1,3] = twist[1]
    tHat[2,3] = twist[2]
    return tHat
