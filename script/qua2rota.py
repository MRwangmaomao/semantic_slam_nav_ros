import numpy as np
from autolab_core import RigidTransform
 
  

orientation = {'y':  0.499547, 'x': -0.49538, 'z': -0.498407 , 'w': 0.506598}
position = {'y': -0.0660151, 'x': 0.22722, 'z': 0.915787}
    
rotation_quaternion = np.asarray([orientation['w'], orientation['x'], orientation['y'], orientation['z']])
translation = np.asarray([position['x'], position['y'], position['z']])
 
T_qua2rota = RigidTransform(rotation_quaternion, translation)

print(T_qua2rota)