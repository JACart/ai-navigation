#!/usr/bin/env python

import transformations

#X
v0 = [[13.9,
-13.2,
35.9,
5.54,
48,
59.1,
6.53], 
#Y
[1.34,
-6.08,
36.3,
71.6,
86.6,
72.1,
-13.9]] 

#Latitude
v1 = [[38.432054,
38.432256,
38.431686,
38.431634,
38.43128,
38.43131,
38.432203],
#Longitude
[-78.876233,
-78.876043,
-78.876154,
-78.875624,
-78.87587,
-78.876074,
-78.876289]]


affine = transformations.affine_matrix_from_points(v0, v1)
print(affine)
