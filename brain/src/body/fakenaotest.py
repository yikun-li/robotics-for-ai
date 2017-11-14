from util.euclid import Vector3 as V3
import fake_nao
import nao
import time
import math
import numpy
import pprint

from util.scale import scale
from motion import *

orig = V3(2, 3, 5)

simnao = nao.Nao('localhost', '54010')
nao = fake_nao.FakeNao('localhost')

nao.init_pose()
simnao.init_pose()

#nao.get_proxy("motion").walkTo(0.0, 0.0, math.pi / 4)
#simnao.get_proxy("motion").walkTo(0.0, 0.0, math.pi / 4)
#
#nao.get_proxy("motion").walkTo(1.5, 0.0, 0*math.pi / 4)
#simnao.get_proxy("motion").walkTo(1.5, 0.0, 0*math.pi / 4)
#
#nao.get_proxy("motion").walkTo(-1.5, 0.0, -math.pi / 4)
#simnao.get_proxy("motion").walkTo(-1.5, 0.0, -math.pi / 4)
#
simnao.get_proxy("motion").setAngles("LArm", [0] * 6, 1)
nao.get_proxy("motion").setAngles("LArm", 0, 1)
simnao.get_proxy("motion").setAngles("RArm", [0] * 6, 1)
nao.get_proxy("motion").setAngles("RArm", 0, 1)


time.sleep(1)

#joints = ['HeadPitch', 'HeadYaw']
joints = ['LWristYaw', 'LElbowRoll', 'LElbowYaw', 'LShoulderRoll', 'LShoulderPitch'] \
       + ['RWristYaw', 'RElbowRoll', 'RElbowYaw', 'RShoulderRoll', 'RShoulderPitch']
#joints = ['LWristYaw', 'LElbowRoll', 'LElbowYaw', 'LShoulderRoll', 'LShoulderPitch']
#joints = ['RWristYaw', 'RElbowRoll', 'RElbowYaw', 'RShoulderRoll', 'RShoulderPitch']
#joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
#joints = ['LHipYawPitch', 'RHipRoll', 'RHipRoll', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
#joints = ['LKneePitch', 'RKneePitch', 'LHipPitch', 'RHipPitch', 'LAnklePitch', 'RAnklePitch', 'RHipRoll', 'LHipRoll']
#joints = ['LKneePitch']
for i in range(10):

    for joint in joints:
        limits = nao.get_proxy("motion").getLimits(joint)[0]
        simlimits = simnao.get_proxy("motion").getLimits(joint)[0]
        val = scale(numpy.random.random(1)[0], 0, 1, limits[0] * 0.8, limits[1] * 0.8)
        simnao.get_proxy("motion").setAngles(joint, val, 1)
        nao.get_proxy("motion").setAngles(joint, val, 1)
        #simnao.get_proxy("motion").setAngles("RKneePitch", val, 1)
        #nao.get_proxy("motion").setAngles("RKneePitch", val, 1)
        print "Setting joint %s to value %.4f" % (joint, val)

    time.sleep(1.5)

    lh_pos = nao.get_proxy("motion").getPosition("RArm", SPACE_NAO, False)
    lh_poss = simnao.get_proxy("motion").getPosition("RArm", SPACE_NAO, False)

    lleg_pos = nao.get_proxy("motion").getPosition("LLeg", SPACE_NAO, False)
    lleg_poss = simnao.get_proxy("motion").getPosition("RLeg", SPACE_NAO, False)
    print "--- left leg ----"
    print "Model: %s" % repr(["%.4f" % x for x in lleg_pos])
    print "Simul: %s" % repr(["%.4f" % x for x in lleg_poss])

    print "---- left arm ----"
    print "Model: %s" % repr(["%.4f" % x for x in lh_pos])
    print "Simul: %s" % repr(["%.4f" % x for x in lh_poss])
    print "Angles:"
    jnts = nao.get_proxy("motion").getJointNames("LArm")
    print dict(zip(jnts, nao.get_proxy("motion").getAngles("LArm", False)))
    print dict(zip(jnts, simnao.get_proxy("motion").getAngles("LArm", False)))
    print "Difference: "
    print repr(["%.4f" % abs(lh_pos[idx] - lh_poss[idx]) for idx in range(0, 6)])
    print ""

time.sleep(1)
