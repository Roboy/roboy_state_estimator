import numpy as np
import math
from pyquaternion import Quaternion

p0 = Quaternion([0,1,0,0])
rot0 = Quaternion([0.7071068, 0, 0, 0.7071068])

p1 = rot0*p0*rot0.inverse
print(p1)