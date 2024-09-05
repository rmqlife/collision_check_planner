import numpy as np
from scipy.spatial.transform import Rotation
from spatialmath import SE3

def rpy2matrix(rpy):

    r = Rotation.from_euler('zyx', rpy, degrees=False)
    return r.as_matrix()

def matrix2quat(matrix):
    r = Rotation.from_matrix(matrix)
    return r.as_quat()

def rpy2quat(rpy):
    r = Rotation.from_euler('zyx', rpy, degrees=False)
    return r.as_quat()

R1 = np.array(  [
  [-0.9531  ,  0.2218   , 0.2058  ,  0.3639   ] ,
  [ 0.3021   , 0.735   ,  0.607  ,  -0.3796    ],
  [-0.01662  , 0.6407  , -0.7676  ,  0.01871   ],
   [0        , 0        , 0      ,   1  ]
]
   )

R2 = np.array([
   [0.9964  ,  0.05054 , -0.0678 ,  -0.5801] ,   
   [0.08369 , -0.7045  ,  0.7048  ,  0.4005  ],
  [-0.01214  ,-0.7079 ,  -0.7062  ,  0.173 ] ,   
   [0       ,  0    ,     0       ,  1 ]   
])


R11 = SE3(  [
   [0.9998  ,  0.01047  ,-0.01769  ,-0.6107    ],
  [-0.005244, -0.7024  , -0.7117   ,-0.05845   ],
  [-0.01988 ,  0.7117  , -0.7022   , 0.1724    ],
   [0       ,  0       ,  0       ,  1 ]
] )


R22 = SE3(
    [
   [0.9964  ,  0.05054 , -0.0678 ,  -0.5801] ,   
   [0.08369 , -0.7045  ,  0.7048  ,  0.4005  ],
  [-0.01214  ,-0.7079 ,  -0.7062  ,  0.173 ] ,   
   [0       ,  0    ,     0       ,  1 ]   
]
)
#R2 = R3*R1

print("R1",matrix2quat(R1[0:3,0:3]),R1[0:3,3])
print("R2",matrix2quat(R2[0:3,0:3]),R2[0:3,3])

print("////////////////")
print(R11.inv() @ R22)


R4 =np.array([
   
   [0.9999  ,  0.01419  ,-0.005593 ,-0.6226    ],
   [0.005845 ,-0.6951   ,-0.7189  , -0.03173  ] ,
  [-0.01409  , 0.7187   ,-0.6951   , 0.1716 ]  , 
   [0       ,  0        , 0        , 1 ]

])

print("R4",matrix2quat(R4[0:3,0:3]),R4[0:3,3]) 
R44 = SE3(
    
[
   
   [0.9999  ,  0.01419  ,-0.005593 ,-0.6226    ],
   [0.005845 ,-0.6951   ,-0.7189  , -0.03173  ] ,
  [-0.01409  , 0.7187   ,-0.6951   , 0.1716 ]  , 
   [0       ,  0        , 0        , 1 ]

]

)
new_R2 = R44 @ R11.inv() @ R22
print("R2",new_R2)
new_R2 = np.array(new_R2)
print(matrix2quat(new_R2[0:3,0:3]),new_R2[0:3,3])



