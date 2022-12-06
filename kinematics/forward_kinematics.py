'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        }

        self.joint_offsets = {
            'HeadYaw': [0, 126.5, 0],
            'HeadPitch': [0, 0, 0],
            'LShoulderPitch': [0, 98, 100],
            'LShoulderRoll': [0, 0, 0],
            'LElbowYaw': [105, 15, 0],
            'LElbowRoll': [0, 0, 0],
            'LWristYaw': [55.95, 0, 0],
            'LHipYawPitch': [0, 50, -85],
            'LHipRoll': [0, 0, 0],
            'LHipPitch': [0, 0, 0],
            'LKneePitch': [0, 0, -100],
            'LAnklePitch': [0, 0, -102.9],
            'LAnkleRoll': [0, 0, 0],
            'RShoulderPitch': [0, 98, -100],
            'RShoulderRoll': [0, 0, 0],
            'RElbowYaw': [105, 15, 0],
            'RElbowRoll': [0, 0, 0],
            'RWristYaw': [55.95, 0, 0],
            'RHipYawPitch': [0, 50, 85],
            'RHipRoll': [0, 0, 0],
            'RHipPitch': [0, 0, 0],
            'RKneePitch': [0, 0, -100],
            'RAnklePitch': [0, 0, -102.9],
            'RAnkleRoll': [0, 0, 0]
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def transformMat4(self, xyz, angle, axis):
        '''transform matrix 4x4

        :param list xyz: the offset of joint
        :param angle: rotation angle in radians
        :param axis: rotation axis
        :return: 4x4 transformation matrix
        '''
        T = identity(4)
        T[0, 3] = xyz[0]
        T[1, 3] = xyz[1]
        T[2, 3] = xyz[2]
        s = np.sin(angle)
        c = np.cos(angle)
        if axis == 'x':
            T[1, 1] = c
            T[1, 2] = -s
            T[2, 1] = s
            T[2, 2] = c
        elif axis == 'y':
            T[0, 0] = c
            T[0, 2] = s
            T[2, 0] = -s
            T[2, 2] = c
        elif axis == 'z':
            T[0, 0] = c
            T[0, 1] = -s
            T[1, 0] = s
            T[1, 1] = c
        return T

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        axis = ' '
        if joint_name.endswith('Roll'):
            axis = 'x'
        elif joint_name.endswith('Pitch'):
            axis = 'y'
        elif joint_name.endswith('Yaw'):
            axis = 'z'
        return self.transformMat4(self.joint_offsets[joint_name], joint_angle, axis)

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                T = T * Tl

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
