import unittest

import navigation.obstacleavoidance.vectorfield
from numpy import zeros

class VectorFieldTestCase(unittest.TestCase):
    def setUp(self):
        self.vectorfield = navigation.obstacleavoidance.vectorfield.VectorField()

        #creat an obstacle matrix of 30x15 with a few obstacles
        self.width = 30
        self.height = 15
        self.matrix = []
        self.matrix = [[0 for col in range(self.width)] for row in range(self.height)]
        self.matrix[10][15] = 1
        self.matrix[10][16] = 1
        self.matrix[10][17] = 1
        self.matrix[10][18] = 1

        self.vectorfield.setObstacleMatrix(self.matrix)
        self.vectorfield.setTarget((1,0)) # full speed, straigt forward

    def tearDown(self):
        self.vectorfield = None

    def test_getObstacleVector(self):
        vector = self.vectorfield.getObstacleVector(18, 10, self.width, self.height)

        #test vector length
        self.assertTrue(vector[0] > 0, "Vector should have a length")

        #test vector direction
        self.assertTrue(vector[1] > 0, "Vector should have a direction")

    def test_affectVector(self):
        obstacleVector = self.vectorfield.getObstacleVector(18, 10, self.width, self.height)

        self.assertFalse(self.vectorfield.obstacleDistanceThreshold < obstacleVector[0], "Please adjust the obstacle in the matrix to a closer position to test the affectVector() function")

        affectVector = self.vectorfield.affectVector(obstacleVector)

        #test vector length
        self.assertTrue(affectVector[0] > 0, "Vector should have a length")

        #test vector direction (should be somewhere leftbehind the robot
        self.assertTrue(affectVector[1] < -90, "Vector should have a direction")

    def test_getAllObstacleAffectVectors(self):
        vectors = self.vectorfield.getAllObstacleAffectVectors(self.matrix)

        self.assertEqual(len(vectors), 4, "Not all affect vectors retreived. Are they close enough?")

        for vector in vectors:
            #test vector length
            self.assertTrue(vector[0] > 0, "Vector should have a length")

            #test vector direction
            self.assertTrue(vector[1] < -90, "Vector should have a direction")

    def test_sumVectors(self):
        vectors = self.vectorfield.getAllObstacleAffectVectors(self.matrix)

        endVector = self.vectorfield.sumVectors(vectors)

        #test vector length
        self.assertTrue(endVector[0] < 0, "Vector should have a length")


def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(VectorFieldTestCase))
    return suite

if __name__ == '__main__':
    unittest.main()
