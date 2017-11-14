import logging
import os
import util.nullhandler
import nao
import math
import numpy

from motion import *

try:
    import cgkit.scene  # To avoid circular deps...
    from cgkit.cgtypes import vec3, mat3, mat4, quat
    import cgkit.euleradapter as euler
except:
    if __name__ == "__main__":
        print "FakeMotion really needs cgkit to do useful work, install if you require this!"

logging.getLogger('Borg.Brain.BodyController.FakeNao').addHandler(util.nullhandler.NullHandler())

class FakeNao(nao.Nao):
    """
    Simulates a Fake NAO
    """

    def __init__(self, robot_ip='localhost', port=9559, nobody=False):
        self.logger = logging.getLogger('Borg.Brain.BodyController.FakeNao')
        self._Nao__robot_ip = robot_ip
        self._Nao__port = port
        self.simulation = True

        self.__username = 'nao'
        self.__password = 'nao'
        self._Nao__FM = FakeFrameManager(self)
        self._Nao__TTS = FakeTTS(self)
        self._Nao__VisionTB = FakeVisionToolbox(self)
        self._Nao__Motion = FakeMotion(self)
        self._Nao__Memory = FakeMemory(self)
        self._Nao__Video = FakeVideo(self)
        self._Nao__BallTracker = FakeBallTracker(self)
        self._Nao__behaviorIDs = {}
        self._Nao__stop_crouch = True
        self._Nao__nobody = nobody

        self.angles = [0] * len(FakeMotion.joints)
        self.position = [0, 0, 0]
        self.orientation = 0

        try:
            dir(cgkit.scene)
        except:
            print "FakeMotion really needs cgkit to do useful work, install if you require this!"


class FakeTTS(object):
    body = None

    class post(object):
        @staticmethod
        def say(text):
            print "[Nao says] " + text
            FakeTTS.body.tts(text)

    def __init__(self, nao):
        self.nao = nao
        from bodycontroller import BodyController
        FakeTTS.body = BodyController()

    def setLanguage(self, lang):
        pass

    def sayToFile(self, text, filename):
        f = open(filename, "a")
        f.write("%s\n" % text)
        f.close()

class FakeMemory(object):
    def __init__(self, nao):
        self.nao = nao

    def getData(self, name):
        return 0.0

class FakeVideo(object):
    def __init__(self, nao):
        self.nao = nao

class FakeBallTracker(object):
    def __init__(self, nao):
        self.nao = nao

class FakeVisionToolbox(object):
    def __init__(self, nao):
        self.nao = nao

class FakeFrameManager(object):
    def __init__(self, nao):
        self.nao = nao
        self.__next_id = 0

    def playBehavior(self, id):
        self.nao.logger.info("Fakeplaying behavior %d" % id)
    
    def completeBehavior(self, id):
        self.nao.logger.info("Fakecompleting behavior %d" % id)

    def newBehavior(self, path, contents):
        cur_id = self.__next_id
        self.__next_id += 1
        return cur_id

    def newBehaviorFromFile(self, file, path):
        return self.newBehavior(None, None)

################################################################################
## Geometry of the different NAO model                                        ##
################################################################################
## This information is taken from:                                            ##
##                                                                            ##
## http://www.aldebaran-robotics.com/documentation/nao/hardware/kinematics/   ##
##        nao-links-32.html                                                   ##
## http://www.aldebaran-robotics.com/documentation/nao/hardware/kinematics/   ##
##        nao-links-33.html                                                   ##
## http://www.aldebaran-robotics.com/documentation/nao/hardware/kinematics/   ##
##        nao-links-40.html                                                   ##
##                                                                            ##
## All measurements are in meters                                             ##
################################################################################
GeometryV32 = { \
    "NeckOffsetZ": 0.12650,
    "ShoulderOffsetY": 0.098,
    "ElbowOffsetY": 0.0,
    "UpperArmLength": 0.090,
    "LowerArmLength": 0.05055,
    "ShoulderOffsetZ": 0.1,
    "HandOffsetX": 0.058,
    "HipOffsetZ": 0.085,
    "HipOffsetY": 0.050,
    "ThighLength": 0.100,
    "TibiaLength": 0.10274,
    "FootHeight": 0.04511,
    "HandOffsetZ": 0.01590,
    "CameraTopOffsetX": 0.0539,
    "CameraTopOffsetY": 0.0,
    "CameraTopOffsetZ": 0.0679,
    "CameraBottomOffsetX": 0.0488,
    "CameraBottomOffsetY": 0.0,
    "CameraBottomOffsetZ": 0.02381,
    "CameraBottomPitch": 0.6981
}

GeometryV33 = { \
    "NeckOffsetZ": 0.1265,
    "ShoulderOffsetY": 0.098,
    "ElbowOffsetY": 0.015,
    "UpperArmLength": 0.105,
    "LowerArmLength": 0.05595,
    "ShoulderOffsetZ": 0.1,
    "HandOffsetX": 0.05775,
    "HipOffsetZ": 0.085,
    "HipOffsetY": 0.050,
    "ThighLength": 0.1,
    "TibiaLength": 0.1029,
    "FootHeight": 0.04519,
    "HandOffsetZ": 0.01231,
    "CameraTopOffsetX": 0.0539,
    "CameraTopOffsetY": 0.0,
    "CameraTopOffsetZ": 0.0679,
    "CameraBottomOffsetX": 0.0488,
    "CameraBottomOffsetY": 0.0,
    "CameraBottomOffsetZ": 0.02381,
    "CameraBottomPitch": 0.6981
}

GeometryV40 = { \
    "NeckOffsetZ": 0.12650,
    "ShoulderOffsetY": 0.098,
    "ElbowOffsetY": 0.015,
    "UpperArmLength": 0.105,
    "LowerArmLength": 0.05595,
    "ShoulderOffsetZ" : 0.1,
    "HandOffsetX": 0.05775,
    "HipOffsetZ": 0.085,
    "HipOffsetY": 0.05,
    "ThighLength": 0.1,
    "TibiaLength": 0.1029,
    "FootHeight": 0.04519,
    "HandOffsetZ": 0.01231,
    "CameraTopOffsetX": 0.05871,
    "CameraTopOffsetY": 0.0,
    "CameraTopOffsetZ": 0.06394,
    "CameraBottomOffsetX": 0.05071,
    "CameraBottomOffsetY": 0.0,
    "CameraBottomOffsetZ": 0.01774,
    "CameraBottomPitch": 0.6929
}

# Vectors representing the axes for convenience
try:
    AXIS_X = vec3(1, 0, 0)
    AXIS_Y = vec3(0, 1, 0)
    AXIS_Z = vec3(0, 0, 1)
except:
    # This means that Vec3 isn't available, but the user is being
    # warned about this anyway so don't bother shouting about it here.
    pass

class FakeMotion(object):
    limits = [  \
        [-2.0856685638427734, 2.0856685638427734, 8.2679738998413086],
        [-0.6719517707824707, 0.51487213373184204, 3.0232594013214111],
        [-2.0856685638427734, 2.0856685638427734, 8.2679738998413086],
        [0.0087266461923718452, 1.6493360996246338, 3.0232594013214111],
        [-2.0856685638427734, 2.0856685638427734, 8.2679738998413086],
        [-1.5620696544647217, -0.0087266461923718452, 3.0232594013214111],
        [-1.8238691091537476, 1.8238691091537476, 5.0314350128173828], 
        [0.0, 1.0, 3.7100000381469727], 
        [-1.1452850103378296, 0.74071770906448364, 4.1617374420166016], \
        [-0.37943458557128906, 0.79045963287353516, 4.1617374420166016], 
        [-1.7737780809402466, 0.48397979140281677, 6.4023914337158203], 
        [-0.092327915132045746, 2.112546443939209, 6.4023914337158203], 
        [-1.1894419193267822, 0.9225810170173645, 6.4023914337158203], 
        [-0.76899206638336182, 0.3977605402469635, 4.1617374420166016], 
        [-1.1452850103378296, 0.74071770906448364, 4.1617374420166016], 
        [-0.73827427625656128, 0.4495968222618103, 4.1617374420166016], 
        [-1.7737780809402466, 0.48397979140281677, 6.4023914337158203], 
        [-0.092327915132045746, 2.112546443939209, 6.4023914337158203], 
        [-1.1863002777099609, 0.93200582265853882, 6.4023914337158203], 
        [-0.38868483901023865, 0.78592175245285034, 4.1617374420166016], 
        [-2.0856685638427734, 2.0856685638427734, 8.2679738998413086], 
        [-1.6493360996246338, -0.0087266461923718452, 3.0232594013214111], 
        [-2.0856685638427734, 2.0856685638427734, 8.2679738998413086], 
        [0.0087266461923718452, 1.5620696544647217, 3.0232594013214111], 
        [-1.8238691091537476, 1.8238691091537476, 5.0314350128173828], 
        [0.0, 1.0, 3.7100000381469727]]

    joints = ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll',
              'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 
              'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 
              'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch', 'RHipRoll', 
              'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 
              'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 
              'RElbowRoll', 'RWristYaw', 'RHand']

    geometry = GeometryV32

    def __init__(self, nao):
        self.nao = nao
        self.__cache = [{}, {}, {}]
        self.__pos_cache = [{}, {}, {}]

    def walkTo(self, X, Y, Theta):
        cur_position = vec3(self.nao.position)
        curTheta = self.nao.orientation

        movement = vec3(X, Y, 0)

        rotation = quat(curTheta, AXIS_Z)
        movement = rotation.rotateVec(movement)

        self.nao.position = list(cur_position + movement)
        self.nao.orientation = (curTheta + Theta) % (math.pi * 2)

    def getTransform(self, name, space, useSensorValues, joints=None):
        # Utilize cache if available
        if name in self.__cache[space]:
            return self.__cache[space][name]

        if not joints:
            joints = dict(zip(self.joints, self.nao.angles))

        mat = mat4.identity()
        larm = ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LArm']
        rarm = ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RArm']
        lleg = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'LLeg']
        rleg = ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 'RLeg']
        if name in larm:
            path = larm[:larm.index(name) + 1]
            for joint in reversed(path):
                if joint == "LShoulderPitch":
                    offset = vec3(0, self.geometry['ShoulderOffsetY'], self.geometry['ShoulderOffsetZ'])
                    mat.rotate(-joints['LShoulderPitch'], AXIS_Y)
                    mat.translate(-offset)
                elif joint == "LShoulderRoll":
                    mat.rotate(-joints['LShoulderRoll'], AXIS_Z)
                elif joint == "LElbowYaw":
                    mat.rotate(-joints['LElbowYaw'], AXIS_X)
                    offset = vec3(self.geometry['UpperArmLength'], self.geometry['ElbowOffsetY'], 0)
                    mat.translate(-offset)
                elif joint == "LElbowRoll":
                    mat.rotate(-joints['LElbowRoll'], AXIS_Z)
                elif joint == "LWristYaw":
                    mat.rotate(-joints['LWristYaw'], AXIS_X)
                    offset = vec3(self.geometry['LowerArmLength'], 0, 0)
                    mat.translate(-offset)
                elif joint == "LHand" or name == "LArm":
                    offset = vec3(self.geometry['HandOffsetX'], 0, -self.geometry['HandOffsetZ'])
                    mat.translate(-offset)
        elif name in rarm:
            path = rarm[:rarm.index(name) + 1]
            for joint in reversed(path):
                if joint == "RShoulderPitch":
                    offset = vec3(0, -self.geometry['ShoulderOffsetY'], self.geometry['ShoulderOffsetZ'])
                    mat.rotate(-joints['RShoulderPitch'], AXIS_Y)
                    mat.translate(-offset)
                elif joint == "RShoulderRoll":
                    mat.rotate(-joints['RShoulderRoll'], AXIS_Z)
                elif joint == "RElbowYaw":
                    mat.rotate(-joints['RElbowYaw'], AXIS_X)
                    offset = vec3(self.geometry['UpperArmLength'], -self.geometry['ElbowOffsetY'], 0)
                    mat.translate(-offset)
                elif joint == "RElbowRoll":
                    mat.rotate(-joints['RElbowRoll'], AXIS_Z)
                elif joint == "RWristYaw":
                    mat.rotate(-joints['RWristYaw'], AXIS_X)
                    offset = vec3(self.geometry['LowerArmLength'], 0, 0)
                    mat.translate(-offset)
                elif joint == "RHand" or name == "RArm":
                    offset = vec3(self.geometry['HandOffsetX'], 0, -self.geometry['HandOffsetZ'])
                    mat.translate(-offset)
        elif name == "Torso":
            pass # No translation
        elif name == "Head":
            offset = vec3(0, 0, self.geometry['NeckOffsetZ'])
            mat.rotate(-joints['HeadPitch'], AXIS_Y)
            mat.rotate(-joints['HeadYaw'], AXIS_Z)
            mat.translate(-offset)
        elif name == "CameraTop":
            offset = vec3(self.geometry['CameraTopOffsetX'],
                          self.geometry['CameraTopOffsetY'],
                          self.geometry['CameraTopOffsetZ'])
            mat.translate(-offset)
            mat *= self.getTransform("Head", SPACE_TORSO, useSensorValues, joints)
        elif name == "CameraBottom":
            mat.rotate(-self.geometry['CameraBottomPitch'], AXIS_Y)
            offset = vec3(self.geometry['CameraBottomOffsetX'],
                          self.geometry['CameraBottomOffsetY'],
                          self.geometry['CameraBottomOffsetZ'])
            mat.translate(-offset)
            mat *= self.getTransform("Head", SPACE_TORSO, useSensorValues, joints)
        elif name in lleg:
            path = lleg[:lleg.index(name) + 1]
            for joint in reversed(path):
                if joint == "LHipYawPitch":
                    hipYP = vec3(0, 1, -1).normalize()
                    mat.rotate(-joints['LHipYawPitch'], hipYP)
                    offset = vec3(0, self.geometry['HipOffsetY'], -self.geometry['HipOffsetZ'])
                    mat.translate(-offset)
                elif joint == "LHipRoll":
                    mat.rotate(-joints['LHipRoll'], AXIS_X)
                elif joint == "LHipPitch":
                    mat.rotate(-joints['LHipPitch'], AXIS_Y)
                elif joint == "LKneePitch":
                    mat.rotate(-joints['LKneePitch'], AXIS_Y)
                    offset = vec3(0, 0, -self.geometry['ThighLength'])
                    mat.translate(-offset)
                elif joint == "LAnklePitch":
                    mat.rotate(-joints['LAnklePitch'], AXIS_Y)
                    offset = vec3(0, 0, -self.geometry['TibiaLength'])
                    mat.translate(-offset)
                elif joint == "LAnkleRoll":
                    mat.rotate(-joints['LAnkleRoll'], AXIS_X)
                elif joint == "LLeg":
                    offset = vec3(0, 0, -self.geometry['FootHeight'])
                    mat.translate(-offset)
        elif name in rleg:
            path = rleg[:rleg.index(name) + 1]
            for joint in reversed(path):
                if joint == "RHipYawPitch":
                    hipYP = vec3(0, -1, -1).normalize()
                    mat.rotate(-joints['RHipYawPitch'], hipYP)
                    offset = vec3(0, -self.geometry['HipOffsetY'], -self.geometry['HipOffsetZ'])
                    mat.translate(-offset)
                elif joint == "RHipRoll":
                    mat.rotate(-joints['RHipRoll'], AXIS_X)
                elif joint == "RHipPitch":
                    mat.rotate(-joints['RHipPitch'], AXIS_Y)
                elif joint == "RKneePitch":
                    mat.rotate(-joints['RKneePitch'], AXIS_Y)
                    offset = vec3(0, 0, -self.geometry['ThighLength'])
                    mat.translate(-offset)
                elif joint == "RAnklePitch":
                    mat.rotate(-joints['RAnklePitch'], AXIS_Y)
                    offset = vec3(0, 0, -self.geometry['TibiaLength'])
                    mat.translate(-offset)
                elif joint == "RAnkleRoll":
                    mat.rotate(-joints['RAnkleRoll'], AXIS_X)
                elif joint == "RLeg":
                    offset = vec3(0, 0, -self.geometry['FootHeight'])
                    mat.translate(-offset)
        else:
            print "Unknown effector : `%s'" % name

        self.__cache[SPACE_TORSO][name] = mat4(mat)

        if space == SPACE_TORSO:
            return mat

        # SPACE_NAO: Need the position of the torso in Nao Space. The origin is
        # located at the average of the positions of the two feet.
        #
        # CAUTION: this doesn't fully work. Origin might be calculated correctly
        # calculated but the orientation is defined such that the X-axis must
        # always look forward so with complicated leg configurations, the
        # coordinate system might not be the exact average between the two
        # orientations. Or something like that. I don't know, I give up ;-)
        lleg = self.getPosition("LLeg", SPACE_TORSO, True)
        rleg = self.getPosition("RLeg", SPACE_TORSO, True)
        lpos = vec3(lleg[:3])

        avg = list(numpy.average([lleg, rleg], 0))

        offset = vec3(avg[:3])

        # Setup transformation matrix
        change = mat4.identity()
        change.setMat3(mat3.fromEulerZYX(avg[5], avg[4], avg[3]))

        mat.translate(offset)
        mat *= change

        self.__cache[SPACE_NAO][name] = mat4(mat)

        if space == SPACE_NAO:
            return mat

        # SPACE_WORLD: The last option. This is the same as SPACE_NAO when
        # naoqi starts, but when the NAO moves about, the origin is left behind.
        #
        # CAUTION: behavior not identical to naoqi (yet). Don't know what the
        # problem is, but the rotation and position reported by a (simulated)
        # naoqi are different from the values calculated here, although they do
        # seem to make sense... Don't known, don't care, give up (for now).
        # Probably naoqi has an estimate of the error when performing walks and
        # incorporates that in the resulting odometry.
        offset = vec3(self.nao.position)
        mat.rotate(-self.nao.orientation, AXIS_Z)
        mat.translate(-offset)

        self.__cache[SPACE_WORLD][name] = mat4(mat)

        return mat

    def killAll(self):
        pass

    def getPosition(self, effectorName, space, useSensorValues):
        if space not in [SPACE_WORLD, SPACE_TORSO, SPACE_NAO]:
            raise Exception("Unexpected task space")

        # Utilize cache if available
        if effectorName in self.__pos_cache[space]:
            return self.__pos_cache[space][effectorName]

        # First calculate in Torso Space
        mat = self.getTransform(effectorName, space, useSensorValues)

        pos, rotation, scaling = mat.inverse().decompose()
        rotation = rotation.inverse().getMat3()

        yaw, pitch, roll = rotation.toEulerXYZ()
        rot = vec3(-yaw, -pitch, -roll)

        self.__pos_cache[space][effectorName] = [pos.x, pos.y, pos.z, rot.x, rot.y, rot.z]

        return [pos.x, pos.y, pos.z, rot.x, rot.y, rot.z]

    def setPosition(self, effectorName, space, position, fractionMaxSpeed, axisMask):
        possible = ['LArm', 'RArm', 'Head', 'LLeg', 'RLeg', 'Torso']
        if effectorName not in possible:
            raise Exception("%s was not recognize as an effector or a sensor" \
                            "name" % effectorName)

        if space not in [SPACE_WORLD, SPACE_TORSO, SPACE_NAO]:
            raise Exception("Unexpected task space")

        self.nao.logger.error("setPosition is not available in FakeNao")

    def changePosition(self, effectorName, space, positionChange, fractionMaxSpeed, axisMask):
        possible = ['LArm', 'RArm', 'Head', 'LLeg', 'RLeg', 'Torso']
        if effectorName not in possible:
            raise Exception("%s was not recognize as an effector or a sensor" \
                            "name" % effectorName)

        if space not in [SPACE_WORLD, SPACE_TORSO, SPACE_NAO]:
            raise Exception("Unexpected task space")

        self.nao.logger.error("changePosition is not available in FakeNao")

    def setAngles(self, names, angles, fractionMaxSpeed):
        if not type(names) is list:
            try:
                names = self.getJointNames(names)
            except:
                if type(names) is str:
                    names = [names]

        if not type(names) is list:
            try:
                names = list(names)
            except:
                names = [names]

        if not type(names) is list:
            raise Exception("%s is not recognized as a valid namelist" % repr(names))
        
        if not type(angles) is list:
            try:
                angles = list(angles)
            except:
                angles = [angles] * len(names)

        # Reset position cache as it will change
        self.__cache = [{}, {}, {}]
        self.__pos_cache = [{}, {}, {}]
        for idx, name in enumerate(names):
            joint_index = self.joints.index(name)
            limits = self.limits[joint_index]
            new_value =  max(min(angles[idx], limits[1]), limits[0])
            self.nao.angles[joint_index] = new_value
            if name == "LHipYawPitch":
                oidx = self.joints.index("RHipYawPitch")
                self.nao.angles[oidx] = -new_value
            elif name == "RHipYawPitch":
                oidx = self.joints.index("LHipYawPitch")
                self.nao.angles[oidx] = -new_value
                
        
    def changeAngles(self, names, changes, fractionMaxSpeed):
        if not type(names) is list:
            try:
                names = self.getJointNames(names)
            except:
                if type(names) is str:
                    names = [names]

        if not type(names) is list:
            try:
                names = list(names)
            except:
                names = [names]

        if not type(names) is list:
            raise Exception("%s is not recognized as a valid namelist" % repr(names))
        
        if not type(changes) is list:
            try:
                changes = list(changes)
            except:
                changes = [changes] * len(names)

        # Reset position cache as it will change
        self.__cache = [{}, {}, {}]
        self.__pos_cache = [{}, {}, {}]
        for idx, name in enumerate(names):
            joint_index = self.joints.index(name)
            cur = self.nao.angles[joint_index]
            limits = self.limits[joint_index]
            new_value = max(min(cur + changes[idx], limits[1]), limits[0])
            self.nao.angles[joint_index] = new_value
            if name == "LHipYawPitch":
                oidx = self.joints.index("RHipYawPitch")
                self.nao.angles[oidx] = new_value
            elif name == "RHipYawPitch":
                oidx = self.joints.index("LHipYawPitch")
                self.nao.angles[oidx] = new_value

    def getAngles(self, names, useSensors):
        if not type(names) is list:
            try:
                names = self.getJointNames(names)
            except:
                if type(names) is str:
                    names = [names]

        if not type(names) is list:
            try:
                names = list(names)
            except:
                names = [names]

        if not type(names) is list:
            raise Exception("%s is not recognized as a valid namelist" % repr(names))
        
        return [self.nao.angles[self.joints.index(joint)] for joint in names]

    def openHand(self, hand):
        if hand not in ['LHand', 'RHand']:
            raise Exception("Unrecognized hand: %s" % hand)

        index = self.joints.index(hand)
        limits = self.limits[iVndex]
        self.nao.angles[index] = limits[0]

    def closeHand(self, hand):
        if hand not in ['LHand', 'RHand']:
            raise Exception("Unrecognized hand: %s" % hand)

        index = self.joints.index(hand)
        limits = self.limits[iVndex]
        self.nao.angles[index] = limits[1]

    def angleInterpolation(self, names, angleLists, timeLists, isAbsolute):
        if type(angleLists) is list and type(angleLists[0]) is list:
            for targetAngles in angleLists:
                self.setAngles(names, targetAngles, maxSpeedFraction)
        else:
            self.setAngles(names, targetAngles, maxSpeedFraction)
            
    def angleInterpolationWithSpeed(self, names, targetAngles, maxSpeedFraction):
        self.setAngles(names, targetAngles, maxSpeedFraction)

    def getJointNames(self, chain):
        if chain == "Body":
            return ['HeadYaw', 'HeadPitch', 'LShoulderPitch', 'LShoulderRoll',
                    'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand', 
                    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 
                    'LAnklePitch', 'LAnkleRoll', 'RHipYawPitch', 'RHipRoll', 
                    'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll', 
                    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 
                    'RElbowRoll', 'RWristYaw', 'RHand']
        elif chain == "LArm":
            return ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw',
                    'LElbowRoll', 'LWristYaw', 'LHand']
        elif chain == "RArm":
            return ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw',
                    'RElbowRoll', 'RWristYaw', 'RHand']
        elif chain == "LLeg":
            return ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
                    'LAnklePitch', 'LAnkleRoll']
        elif chain == "RLeg":
            return ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch',
                    'RAnklePitch', 'RAnkleRoll']
        elif chain == "Head":
            return ['HeadYaw', 'HeadPitch']
        raise Exception('Could not parse "%s" as a valid joint name, chain' \
                        'name or "Body"')

    def getLimits(self, name):
        try:
            joints = self.getJointNames(name)
        except:
            joints = [name]

        return [self.limits[self.joints.index(joint)] for joint in joints]

    def stiffnessInterpolation(self, *args, **kwargs):
        """Fake NAO is always stiff, so noop"""
        pass

