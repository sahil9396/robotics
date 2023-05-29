import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import time
import pybullet_data
import cv2

width = height = 400

fov = 60
aspect = width / height
near = 0.02
far = 5

physicsClient = p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("cuboid.urdf", [0,1.5, 0])
p.setGravity(0, 0, -20)
p.changeDynamics(boxId, -1, 0)
sphereId = p.loadURDF("sphere.urdf", [0, 0, 1.5])

for i in range(100000):
    p.stepSimulation()
    time.sleep(1. / 240.)

    cubePos, cubeOrn = p.getBasePositionAndOrientation(sphereId)
    view_matrix = p.computeViewMatrix([0, 0, cubePos[2]], [0, 1, cubePos[2]], [0, 0, 2])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    images = p.getCameraImage(width, height, view_matrix, projection_matrix,shadow=True, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.

    cv2.imshow('rgb', rgb_opengl)
    
    if cv2.waitKey(1) & 0xFF == ord('q') and( cubePos[2] <= 0.45):
        TextID = p.addUserDebugText("!!Hurry!!", [0.5, 0, 1], [0, 0, 1], 1, 0,p.getQuaternionFromEuler([1.57, 0, 0]))
        time.sleep(2)
        break

    if cubePos[2] <= 0.45:
        p.changeDynamics(sphereId, -1, 0)
        p.removeBody(sphereId)
        p.loadURDF("sphere.urdf", [0, 0, 2])
        if cv2.waitKey(0) & 0xFF == ord('q'):
            TextID = p.addUserDebugText("!!Hurry!!", [0.5, 0, 1], [0, 0, 1], 1, 0,p.getQuaternionFromEuler([1.57, 0, 0]))
            # time.sleep(2)
        time.sleep(2)
        break

p.disconnect()
cv2.destroyAllWindows()
