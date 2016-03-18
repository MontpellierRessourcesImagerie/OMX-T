import numpy as np
import ctypes
from multiprocessing.heap import BufferWrapper

array1 = np.zeros(6, dtype='uint16', order = 'C')

print array1

array1[1] = 5

array2 = np.copy(array1)
array3 = array1.reshape((3,2))

print array2

array1[2] = 3

print array1

print array2

print array3

array2 = array2.reshape((2,3))

print array2

print array3

print array1.ctypes.data

array4 = np.ndarray(shape=[1,3], dtype='uint16', strides=[20, 3], buffer=array1)

print array4.ctypes.data

array5 = array4.copy()

print array5

print array5.ctypes.data

#del array4

print array1

