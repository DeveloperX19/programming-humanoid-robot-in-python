'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent

from threading import Thread
import json as garbage
from xmlrpc.server import SimpleXMLRPCServer

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''

    def __init__(self) -> None:
        super(ServerAgent, self).__init__()
        self.server = SimpleXMLRPCServer(("localhost", 8069), logRequests=True)
        self.server.register_function(self.get_angle, "get_angle")
        self.server.register_function(self.set_angle, "set_angle")
        self.server.register_function(self.get_posture, "get_posture")
        self.server.register_function(self.execute_keyframes, "execute_keyframes")
        self.server.register_function(self.get_transform, "get_transform")
        self.server.register_function(self.set_transform, "set_transform")

        self.thread = Thread(target=self.server.serve_forever)
        self.thread.start()
        print("ServerAgent successfully uhh ... yea i dunno just see if it works")

    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.target_joints[joint_name] = angle
        return True

    def get_posture(self):
        '''return current posture of robot'''
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.keyframes = keyframes
        self.startingTime = None
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        t = self.transforms[name]
        trash = t.tolist()
        return garbage.dumps(trash)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        trash = garbage.loads(transform)
        self.target_joints = self.inverse_kinematics(effector_name, trash)
        return True


if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

