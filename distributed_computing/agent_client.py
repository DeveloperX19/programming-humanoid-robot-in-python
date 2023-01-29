'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref

import xmlrpc.client

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # na bruh

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # not happening


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    def __init__(self):
        self.post = PostHandler(self)
        self.server = xmlrpc.client.ServerProxy("http://localhost:8069/")
        print("ClientAgent up... (hopefully)")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        try:
            return self.server.get_angle(joint_name)
        except:
            print("get_angle just couldn't")
            return 0
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        try:
            return self.server.set_angle(joint_name, angle)
        except:
            print("set_angle is a failure")
            return False

    def get_posture(self):
        '''return current posture of robot'''
        try:
            return self.server.get_posture()
        except:
            print("get_posture exploded")
            return "Unknown"

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        try:
            return self.server.execute_keyframes(keyframes)
        except:
            print("execute_keyframes took the L")
            return False

    def get_transform(self, name):
        '''get transform with given name
        '''
        try:
            return self.server.get_transform(name)
        except:
            print("get_transform died")
            return False

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        try:
            return self.server.set_transform(effector_name, transform)
        except:
            print("set_transform forgor")
            return False

if __name__ == '__main__':
    agent = ClientAgent()