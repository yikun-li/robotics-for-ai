
'''this is not for unit testing, just manual testing as long as the bodycontroller etc is not ready yet!!'''

import allbehaviors
import memory
import time

def testall():

    #create a gpsr behavior:
    ab = allbehaviors.AllBehaviors()
    testB = ab.gpsrc({})

    #do a lot of test updates:
    for i in range(1,100):
	addPerception(i)
        testB.update()
        if testB.is_finished():
                break
        if testB.is_failed():
                break


def addPerception(i):
	m = memory.Memory()

	if (i == 3):
		m.add_item("testobject", time.time(), {'a':'x'})
		#TODO: add something to memory here....

	#TODO: maybe make this a list of tuples (i,wat er in het memory moet)
	#en verwerk die dan

if __name__ == "__main__":
    testall()
