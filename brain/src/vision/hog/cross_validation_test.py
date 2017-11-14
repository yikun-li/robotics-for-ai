'''
Created on Jun 4, 2012

@author: amir
'''
import unittest
import cross_validation
import os
import shutil
class CrossValidation_test(unittest.TestCase):
    def setUp(self):
        
        self.raddress = os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/cs') + '/'
        self.waddress = os.path.abspath(os.environ['BORG'] + '/Brain/data/hog_test/cswrite') + '/'
        try:
            os.mkdir(os.environ['BORG'] + '/Brain/data/hog_test/cswrite')
        except:
            pass
        os.system("rm -rf "+ self.waddress+"*.*")
        self.k_fold = 3
        
        
        
    def tearDown(self):
        shutil.rmtree(os.environ['BORG'] + '/Brain/data/hog_test/cswrite')
        
    def test_readFolder(self):
        cs = cross_validation.CrossValidation(self.raddress, self.waddress, self.k_fold)
        data = cs.readFolder()
        
        self.assertEqual(len(data), 9, "The number of classes is not equal to the number of folders")
        self.assertEqual(os.path.isfile(os.path.join(self.raddress,'data-label.dat')),True, "The data-label file is not written.")
    def test_writeFolder(self):
        cs = cross_validation.CrossValidation(self.raddress, self.waddress, self.k_fold)
        cs.readFolder()
        data = cs.writeFolder()
        self.assertEqual(9, data, "The number of files are not equal to the actual number of files in folders")
    
    def test_validation(self):
        pass
    
    def test_dataShuffler(self):
        testData = ['/staf/amir/test/1.png', '/staf/amir/test/2.png', '/staf/amir/test/3.png','/staf/amir/test/2.png', '/staf/amir/test/3.png', '/staf/amir/test/2.png', '/staf/amir/test/3.png']
        length = len(testData)
        cs = cross_validation.CrossValidation(self.raddress, self.waddress, self.k_fold)
        testShuffle = cs.dataShuffler(testData)
        self.assertEqual(length, len(testShuffle), "The lenght of the input and output differs")
        
    def test_dataOrganizer(self):
        cs = cross_validation.CrossValidation(self.raddress, self.waddress, self.k_fold)
        cs.readFolder()
        cs.writeFolder()
        cs.dataOrganizer()
        
    def test_setFolds(self):
        testSliced = [['/staf/amir/robotica/Brain/data/hog_test/cswrite/1.jpg'],['/staf/amir/robotica/Brain/data/hog_test/cswrite/4.jpg'],['/staf/amir/robotica/Brain/data/hog_test/cswrite/5.jpg']]
        cs = cross_validation.CrossValidation(self.raddress, self.waddress, 3)
        cs.readFolder()
        cs.writeFolder()
        set = cs.setFolds(testSliced)
        
        for i in range(3):
            testList = os.listdir('/staf/amir/robotica/Brain/data/hog_test/cswrite/'+str(i)+"/testset")
            trainingList = os.listdir('/staf/amir/robotica/Brain/data/hog_test/cswrite/'+str(i)+"/trainingset")
            for tests in testList:
                for trains in trainingList:
                    self.assertNotEqual(tests,trains, "The file in the test set is equal to the file in train set")
    def test_writeFolds(self):
        
        cs = cross_validation.CrossValidation(self.raddress, self.waddress, 3)
        cs.readFolder()
        cs.writeFolder()
        
        testSet = ['/staf/amir/robotica/Brain/data/hog_test/cswrite/1.jpg','/staf/amir/robotica/Brain/data/hog_test/cswrite/1320757889.jpg']
        trainingSet = ['/staf/amir/robotica/Brain/data/hog_test/cswrite/4.jpg','/staf/amir/robotica/Brain/data/hog_test/cswrite/5.jpg']
        for i in range(3):
            cs.writeFolds(i, testSet, trainingSet)
            testList = os.listdir('/staf/amir/robotica/Brain/data/hog_test/cswrite/'+str(i)+"/testset")
            trainingList = os.listdir('/staf/amir/robotica/Brain/data/hog_test/cswrite/'+str(i)+"/trainingset")
            self.assertNotEqual(testList,[], "Test directory is empty")
            self.assertNotEqual(trainingList,[], "Training directory is empty")
            for tests in testList:
                for trains in trainingList:
                    self.assertNotEqual(tests,trains, "The file in the test set is equal to the file in train set")
        
 
def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(CrossValidation_test))
    return suite

if __name__ == '__main__':
    unittest.main() 