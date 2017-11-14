import unittest
import time
import random
import logging

from util.threadedsocket import ThreadedSocket
import configparse
import visioncontroller
import memory
import util.loggingextra

class VisionController_TestCase(unittest.TestCase):
    """
    TestCase for the VisionController that manages all communication
    with the vision laptops, the communicators and the vision modules
    """

    def setUp(self):
        if not hasattr(self, "random_seed"):
            random.seed()
            self.random_seed = True
        self.communicatorPort = random.randint(49152, 65535)
        config_dict = configparse.ParameterDict()
        config_dict.add_option("vision_controller", "start_port", "50000")
        config_dict.add_option("vision_controller", "end_port", "52000")
        config_dict.add_option("vision_controller", "communicator_port", self.communicatorPort)
        self.config = config_dict
        self.controller = visioncontroller.VisionController()
        self.controller.set_config(self.config)

    def tearDown(self):
        self.config = None
        self.controller.stop_connections()
        self.controller.__del__()
        self.controller = None

    def test_recognizable_objects(self):
        m = memory.Memory()
        self.assertFalse(m.is_object_recognizable("chair"))

        self.controller.add_recognizable_object("chair", "vision_module", None)
        self.assertTrue(m.is_object_recognizable("chair"))

    def test_observations(self):
        observation = {"object": "chair", "x": 10, "y": 15}
        self.controller.add_observation(observation, "vision_module", ("localhost", 50000))

        self.controller.update()
        m = memory.Memory() 
        obs = m.get_last_observation("vision.unknown")
        self.assertFalse(obs is None, "Got no observation in memory")
        observation['identifier'] = "vision_module"
        observation['source'] = ("localhost", 50000)
        self.assertEquals(observation, obs[1])
        self.assertTrue(obs[0] > time.time() - 0.1)

        observation = {"object": "chair", "x": 10, "y": 15, 'name':'example_name'}
        self.controller.add_observation(observation, "vision_module", ("localhost", 50000))
        self.controller.update()
        obs = m.get_last_observation('example_name')
        self.assertFalse(obs is None)
        self.assertTrue(obs[0] > time.time() - 0.1)

    def test_modules(self):
        # Need to add modules, so recreate object
        self.config.add_option("vision_controller", "modules", "hostname1 = mod1 mod2 mod3\nhostname2 = mod3 mod2 mod1")
        self.config.add_option("vision_controller", "modules_settings", "mod1 = module1\nmod2 = module2\nmod3 = module3")
        self.controller = visioncontroller.VisionController()
        self.controller.set_config(self.config)

        mods1 = self.controller.get_modules("hostname1")
        mods2 = self.controller.get_modules("hostname2")
        self.assertEquals(len(mods1), 3, "Not all modules are started on host 1")
        self.assertEquals(len(mods2), 3, "Not all modules are started on host 1")
        for h in ['module1', 'module2', 'module3']:
            found = False
            for mod in mods1:
                if mod['command'] == h:
                    found = True
                    break
            self.assertTrue(found, "Module " + h + " was not found for host 1")
        for h in ['module1', 'module2', 'module3']:
            found = False
            for mod in mods2:
                if mod['command'] == h:
                    found = True
                    break
            self.assertTrue(found, "Module " + h + " was not found for host 2")

    def test_send_command(self):
        # Set up test communicator
        comm_s = None
        mod_s = None
        client = None
        try:
            #comm_s = ThreadedSocket("", self.communicatorPort)
            comm_s = ThreadedSocket("", "communicator")
            self.assertTrue(comm_s.wait_listen(), "Fake Communicator socket did not open")
            
            self.config.add_option("vision_controller", "modules", "localhost = mod1")
            self.config.add_option("vision_controller", "modules_settings", "mod1 = test")
            self.controller = visioncontroller.VisionController()
            self.controller.set_config(self.config)

            # On the fake communicator, receive the module to start and the port
            # to start it on, because we need that port to connect to.
            client = comm_s.wait_connect()
            self.assertTrue(client != None, "Did not get a connection")
            
            recv = client.wait_data()
            self.assertTrue(recv, "Did not receive any data")

            port = None
            for i in recv:
                if "port" in i:
                    port = i['port']
            
            self.assertTrue(port != None, "Did not receive a port number to connect to")

            # Set up a connection to the port specified by the vision
            # controller, it should be listening on that by now.
            mod_s = ThreadedSocket("localhost", port)
            mod_s.start()
            self.assertTrue(mod_s.wait_connect(), "Fake vision module did not connect")

            # First receive the send_capabilities command that should be send
            # automatically.
            recv = mod_s.wait_data()
            self.assertEquals(len(recv), 1, "Should have received one command")
            data = recv[0]
            self.assertTrue("command" in data, "There should be a command in the received dictionary")
            self.assertEquals(data["command"], "send_capabilities", "Testcommand should be send_capabilities")
            self.assertTrue("params" in data, "There should be command parameters in the received dictionary")
            self.assertEquals(data["params"], {}, "Test parameters should be an empty dictionary")

            # Try sending a test command and receive it
            testCommand = "test_command"
            testParams = {"param1": "test_param"}
            self.controller.send_command("localhost", "mod1", testCommand, testParams)

            # See if the test command has been received
            recv = mod_s.wait_data()
            self.assertEquals(len(recv), 1, "Should have received one command")
            data = recv[0]
            self.assertTrue("command" in data, "There should be a command in the received dictionary")
            self.assertEquals(data["command"], testCommand, "Testcommand should be %s" % testCommand)
            self.assertTrue("params" in data, "There should be command parameters in the received dictionary")
            self.assertEquals(data["params"], testParams, "Test parameters should be %s" % repr(testParams))

        finally:
            if comm_s:
                comm_s.close()
            if mod_s:
                mod_s.close()
            if client:
                client.close()
            self.controller.stop_connections()

def suite():
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(VisionController_TestCase))
    return suite

if __name__ == '__main__':
    logging.getLogger('Borg.Brain').addHandler(util.loggingextra.ScreenOutput())
    logging.getLogger('Borg.Brain').setLevel(logging.INFO)
    unittest.main()
