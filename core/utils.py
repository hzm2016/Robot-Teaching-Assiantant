import numpy as np
from scipy.spatial.distance import cdist
import munkres


def load_class(class_name):
    """[summary]

    Args:
        class_name ([str): name of the class to be built

    Returns:
        [type]: [description]
    """
    components = class_name.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

def squarify(M,val=10000):
    (a,b)=M.shape

    if a == b:
        return M

    if a>b:
        padding=((0,0),(0,a-b))
    else:
        padding=((0,b-a),(0,0))

    return np.pad(M,padding,mode='constant',constant_values=val)

def hungarian_matching(a, b):
    
    m = munkres.Munkres()
    dist = cdist(a,b)
    dist = squarify(dist)
    indices = m.compute(dist)
    
    return indices

if __name__ == "__main__":

    a = np.array([(1, 2),(3,4),(5,6)])
    b = np.array([(3, 4),(7,8)])

    indices = hungarian_matching(a, b)
    print(indices)
