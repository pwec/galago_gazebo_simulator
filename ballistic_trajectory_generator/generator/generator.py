#!/usr/bin/env python
from numpy import sqrt, sin

NAME = 'ballistic_trajectory_generator'

# import services definition
from ballistic_trajectory_generator.srv import *
import rospy 
import numpy as np

class CONSTANTS:
    g = 9.81  # m/s^2
    PI = 3.1415



def generate(req):
    print("Returning [%s]" % (req.begin.position.x))
    
 
    

    traj = TrajectoryResponse()
    traj.trajectory.header.frame_id = "aaa"
    return traj

def GenerateTrajectoryServer():
    p1 = np.array([0, 0, 0])
    p2 = np.array([1, 0, 0])
    
    a = np.array([0, 0, -CONSTANTS.g])
    v = np.array([0, 0, 0])
    x = np.array([0, 0, 0])
   #TODO cannot be distance between point - must be projected on X-Y plane!
    R = numpy.linalg.norm( p2[0:2]- p1[0:2])
    theta = (45 / 180) * CONSTANTS.PI
    # v0 vector length
    #another method: V0 = sqrt(+/- (-x^2 * g)/(y-tan(theta)*2*cos(theta)^2)
    V0 = (2 * R * CONSTANTS.g) / (sqrt(4 * R * g * sin(2 * theta)))
    # v0 vector
    v0 = (V0 * cos(theta), 0, V0 * sin(theta))
    h = (V0 ^ 2) / (2 * CONSTANTS.g)
    
    
    
    rospy.init_node(NAME)
    s = rospy.Service('generate', Trajectory, generate)
    rospy.spin()

if __name__ == "__main__":
    GenerateTrajectoryServer()
