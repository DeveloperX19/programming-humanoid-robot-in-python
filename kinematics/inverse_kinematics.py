'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from scipy.linalg import pinv
from scipy.spatial.transform import Rotation


class InverseKinematicsAgent(ForwardKinematicsAgent):

    def raw_from_mat(self, transform):
        rot = Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')
        tx = transform[3, 0]
        ty = transform[3, 1]
        tz = transform[3, 2]
        rx = rot[0]
        ry = rot[1]
        rz = rot[2]
        return [tx, ty, tz, rx, ry, rz]

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joints = self.chains[effector_name]
        N = len(joints)
        angles = np.random.random(N) * 1e-5
        target = self.raw_from_mat(transform)
        delta = 1
        max_error = 0.1
        for _ in range(1000):
            angle_dict = self.perception.joint
            for i in range(N):
                angle_dict[joints[i]] = angles[i]
            self.forward_kinematics(angle_dict)
            Ts = [self.raw_from_mat(self.transforms[joint])
                  for joint in joints]
            Te = np.matrix(Ts[-1]).T
            e = np.matrix(target) - Te
            e[e > max_error] = max_error
            e[e < -max_error] = -max_error
            T = np.matrix(Ts).T
            J = Te - T
            dT = Te - T
            J[0, :] = dT[2, :] - dT[1, :]
            J[1, :] = dT[0, :] - dT[2, :]
            J[2, :] = dT[1, :] - dT[0, :]
            J[3, :] = 1
            J[4, :] = 1
            J[5, :] = 1
            delta_angles = delta * pinv(J) * e
            angles += np.asarray(delta_angles.T)[0]
            if np.linalg.norm(e) < 1e-5:
                break
        return angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        angles_from = self.perception.joint
        angles_to_raw = self.inverse_kinematics(effector_name, transform)
        joints = self.chains[effector_name]
        angles_to = {}
        for name in angles_from.keys():
            angles_to[name] = angles_from[name]
        for i in range(len(joints)):
            angles_to[joints[i]] = angles_to_raw[i]

        names = self.chains[effector_name]
        times = []
        keys = []
        for name in names:
            times.append([1.0, 7.0])
            keys.append([[angles_from[name], [3, 0.0, 0.0], [3, 0.0, 0.0]], [
                        angles_to[name], [3, 0.0, 0.0], [3, 0.0, 0.0]]])

        self.keyframes = (names, times, keys)


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[3, 1] = 0.05
    T[3, 2] = 0.26
    T[3, 0] = -.4
    agent.set_transforms('LLeg', T)
    agent.run()
