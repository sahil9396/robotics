# import pybullet as p
# import time
# import pybullet_data

# physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
# p.setGravity(0,0,-10)

# planeId = p.loadURDF("plane.urdf")
# cubeStartPos = [0, 0, 1]
# cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# boxId = p.loadURDF("one_shape.urdf", cubeStartPos, cubeStartOrientation)
# for i in range(10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos, cubeOrn)
# p.disconnect()

# sliders

# import pybullet as p
# import pybullet_data

# p.connect(p.GUI)  # or p.SHARED_MEMORY or p.DIRECT
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.loadURDF("plane.urdf")

# # p.loadURDF("sphere.urdf", [1, 0, 1])
# sliderID = p.addUserDebugParameter("Position_x", -15, 15, 0)
# new_pos_x = 0
# while(1):
#     old_pos_x = new_pos_x
#     new_pos_x = p.readUserDebugParameter(sliderID)
#     if(old_pos_x!=new_pos_x):
#         p.loadURDF("sphere.urdf", [new_pos_x, 0, 1])
#     p.stepSimulation()
#     time.sleep(1./240.)
# p.disconnect()


# mouse

# import pybullet as p
# import pybullet_data

# p.connect(p.GUI)  # or p.SHARED_MEMORY or p.DIRECT
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.loadURDF("plane.urdf")
# p.setGravity(0, 0, -10)

# # Watch the spheres spawning as you move the pointer
# while (1):
#     output = p.getMouseEvents()
#     for t, x, y, i, s in output:
#         if(t==1):
#             p.loadURDF("sphere.urdf", [x/200, y/200, 1])
            
#     p.stepSimulation()

# p.disconnect()



# keyboard

# import pybullet as p
# import pybullet_data

# p.connect(p.GUI)  # or p.SHARED_MEMORY or p.DIRECT
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.loadURDF("plane.urdf")
# p.setGravity(0, 0, -10)

# pos = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1]]

# # Spheres will be spawned at various positions depending on the pressed key
# while (1):
#     keys = p.getKeyboardEvents()
#     for k, v in keys.items():
#         if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
#             ID = p.loadURDF("sphere.urdf", pos[0])

#         if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
#             ID = p.loadURDF("sphere.urdf", pos[1])

#         if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
#             ID = p.loadURDF("sphere.urdf", pos[2])

#         if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
#             ID = p.loadURDF("sphere.urdf", pos[3])
            
#     p.stepSimulation()

# p.disconnect()

######################

import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import time
import pybullet_data
import cv2
direct = p.connect(p.GUI)  #, options="--window_backend=2 --render_device=0")
#egl = p.loadPlugin("eglRendererPlugin")

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')
p.loadURDF("cuboid.urdf", [0, 0, 1])
(p.setGravity(0,0,-1))

width = 512
height = 512

fov = 60
aspect = width / height
near = 0.02
far = 5
# camera position , camera target position , up-camera
view_matrix = p.computeViewMatrix([0, 0, 2], [0, 0, 1], [0, 0, 1])

# flied of view, the ratio of width and height , the nearest distance to take image , the farest distance to take image
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

# Get depth values using the OpenGL renderer
images = p.getCameraImage(width,
                          height,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
cv2.imshow('rgb',rgb_opengl)
cv2.waitKey(0)
cv2.destroyAllWindows()





########################

# import pybullet as p
# import time
# import pybullet_data

# physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
# planeId = p.loadURDF("plane.urdf")


# boxId = p.loadURDF("cuboid.urdf", [0, 0, 1])

# (p.setGravity(0,0,-1))
# p.changeDynamics(boxId, -1 ,0)
# heigth =0.5
# sphereId= p.loadURDF("sphere.urdf", [0, 0, 2])
# for i in range(10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
#     cubePos, cubeOrn = p.getBasePositionAndOrientation(sphereId)
    
#     if(cubePos[2] <= heigth):
#         p.changeDynamics(sphereId , -1 , 0)
#         sphereId= p.loadURDF("sphere.urdf", [0, 0, 2])
#         TextID = p.addUserDebugText("!!Hurry!!", [0.5,0,1], [0,0,1], 1,0, p.getQuaternionFromEuler([1.57,0,0])) #text,coordintaes,rgb,size,timetoshow
#         time.sleep(3)
#         break
# # print(cubePos, cubeOrn)
# # text()
# # ah = input()
# p.disconnect()
