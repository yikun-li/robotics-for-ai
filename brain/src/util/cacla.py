import os
import numpy
import math
import util.scale as scale

from util.nn_ensemble import libfann, Ensemble

################################################################################
## CONSTANTS                                                                  ##
################################################################################
EXPLORE_GREEDY = 1     #: E-Greedy Exploration: probability of random action
EXPLORE_GAUSSIAN = 2   #: Gaussian exploration around current estimate

class CACLA(object):
    """
    This class implements the Continuous Action Critic Learning Automaton
    (CACLA), as described by van Hasselt & Wiering (2005). It is a continuous
    implementation fo reinforcement learning. After setting up the parameters,
    you can call the run method with the reward obtained in the current state
    and a description of the current state. 
    """

    def __init__(self, base_path, num_outputs, **kwargs):
        """
        Set up all parameters for the CACLA algorithm to their defaults. They
        can be changed by the set_parameters method or specified as arguments
        for the constructor. One parameter is required for the constructor:

        base_path           The path where CACLA should save its networks and
                            progress reports.
        """

        ########################################################################
        ## GENERAL PARAMETERS                                                 ##
        ########################################################################
        self.actor_file = os.path.join(base_path, "motion_float.net")
        self.critic_file = os.path.join(base_path, "critic_float.net")
        self.progress_file = os.path.join(base_path, "progress")
        self.reward_file = os.path.join(base_path, "total_reward")

        self.ensemble_size = 3
        self.num_hiddens = 200
        self.random_range = 0.1
        self.magnitude = 300            # The maximum absolute value of the
                                        # output of the critic network.
                                        # FANN seems to be limited to (-300,300)
        self.num_outputs = num_outputs

        ########################################################################
        ## CACLA PARAMETERS                                                   ##
        ########################################################################
        self.alpha = 0.01               # Learning rate for updating the Critic
        self.min_alpha = 0.0001         # Minimum learning rate for the Critic
        self.beta = 0.01                # Learning rate for the Actor
        self.min_beta = 0.0001          # Minimum learning rate for the Actor
        self.epsilon = 0.2              # Epsilon for e-greedy exploration
        self.min_epsilon = 0.1          # Minimum exploration
        self.sigma = 2.0                # Standard Deviation for Gaussian
                                        # exploration
        self.min_sigma = 0.2            # Minimum exploration
        self.discount_factor = 0.99     # Decay of the reward
        self.learning_decay = 1.0       # Decay of the learning rates
        self.td_var = 1.0               # Starting variance
        self.var_beta = 0.001           # Factor of running average of variance

        ########################################################################
        ## EXPLORATION PARAMETERS                                             ##
        ########################################################################
        self.random_decay = 0.999       # Decay of the epsilon and sigma factors
        self.explore_strategy = EXPLORE_GAUSSIAN # Exploration strategy
        self.progress = 0               # Number of iterations already  
                                        # performed, useful when restarting.
        try:
            self.progress = int(open(self.progress_file, "r").read())
            print "[%s] Continuing from step %d" \
                  % (self.progress_file, self.progress)
        except:
            self.progress = 0

        ########################################################################
        ## STORAGE FOR USE BY ALGORITHM                                       ##
        ########################################################################
        self.last_output = None
        self.test_performance = False
        self.last_value = None
        self.last_state = None
        self.total_reward = 0
        self.steps = 0

        if kwargs:
            self.set_parameters(**kwargs)

    def reset(self):
        """
        This method will reset the CACLA algorithm, meaning it will be in the
        first state of a sequence. During the first round, no training will
        occur and only an action will be generated.
        """
        self.last_output = None
        self.last_value = None
        self.last_state = None
        if self.test_performance:
            rew_file = os.path.basename(self.reward_file)
            rew_dir = os.path.dirname(self.reward_file)
            rew_file = os.path.join(rew_dir, "test_" + rew_file)
            open(rew_file, 'a').write("%d %f\n" % (self.test_count, self.total_reward))
        else:
            open(self.reward_file, 'a').write("%d %f\n" % (self.progress, self.total_reward))
        self.total_reward = 0
        self.steps = 0

    def set_test(self, test):
        self.test_performance = test
        self.test_count = 0
        if test:
            rew_file = os.path.basename(self.reward_file)
            rew_dir = os.path.dirname(self.reward_file)
            rew_file = os.path.join(rew_dir, "test_" + rew_file)

            prog_file = os.path.basename(self.progress_file)
            prog_dir = os.path.dirname(self.progress_file)
            prog_file = os.path.join(prog_dir, "test_" + prog_file)

            open(rew_file, 'a').write("------ NEW TEST ------\n")
            open(prog_file, 'a').write("------ NEW TEST ------\n")

    def max_reward(self):
        """
        This method will return the maximum reward that should be awarded in any
        non-final state. Only in very rare cases, this value should be exceeded,
        otherwise the networks will quickly saturate the value function.
        """
        return self.magnitude * (1 - self.discount_factor)

    def set_parameters(self, **kwargs):
        """
        Set the parameters for the algorithm. Each parameter may be specified as
        a keyword argument. Available parameters are:

        alpha               The learning rate for the Critic in [0, 1]
        beta                The learning rate for the Actor in [0, 1]
        epsilon             The exploration probability for 
                            e-greedy exploration in [0, 1]
        sigma               The standard deviation for Gaussian exploration > 0
        discount_factor     The value attributed to future rewards in (0, 1)
        random_decay        The decay of the epsilon and sigma paramters after
                            each save of the algorithm. This is the factor
                            the value is multiplied with. Should be in [0, 1]
        explore_Strategy    EXPLORE_GAUSSIAN or EXPLORE_GREEDY
        num_outputs         The number of outputs required from the actor. This
                            should be an integer greater than 0.
        ensemble_size       The number of Neural Networks to use in ensemble to
                            optimize the output of the actor and critic. The
                            output of these networks is averaged to obtain the
                            next action.
        td_var              The initial variance of the TD-error. Default 1.0
        var_beta            The factor of the update of the running average of the
                            varianceof the TD-error. Default: 0.001
        """
        for key, value in kwargs.iteritems():
            if key == "explore_strategy":
                if value == EXPLORE_GAUSSIAN or \
                   value == EXPLORE_GREEDY:
                   self.explore_strategy = value
                else:
                    raise Exception("Invalid exploration strategy %d" % value)
            elif key == "num_outputs":
                self.num_outputs = int(value)
            elif key == "ensemble_size":
                self.ensemble_size = int(value)
            elif key == "sigma" and type(value) is tuple:
                self.min_sigma, self.sigma = value
            elif key == "epsilon" and type(value) is tuple:
                self.min_epsilon, self.epsilon = value
            elif key == "alpha" and type(value) is tuple:
                self.alpha, self.min_alpha = value
            elif key == "beta" and type(value) is tuple:
                self.beta, self.min_beta = value
            elif key in self.__dict__:
                self.__dict__[key] = float(value)
            elif not key in self.__dict__:
                raise Exception("Unknown setting: %s = %s" % (key, repr(value)))

    def setup_ann(self, nn_input):
        """
        This function loads existing networks or creates new networks for the
        actor and the critic.
        """
        activation_func = libfann.SIGMOID_SYMMETRIC
        try:
            num_inputs = int(nn_input)
        except:
            num_inputs = len(nn_input)

        # Set up actor network
        self.actor = Ensemble(self.ensemble_size)
        if self.actor.create_from_file(self.actor_file):
            print "Actor Neural network loaded from %s" % self.actor_file
        else:
            # Loading failed, create a new Actor network
            self.actor = Ensemble(self.ensemble_size)
            self.progress = 0
            layers = [num_inputs, self.num_hiddens, self.num_outputs]
            self.actor.create_standard_array(layers)
            self.actor.randomize_weights(-self.random_range, self.random_range)
            self.actor.set_activation_function_hidden(activation_func)
            self.actor.set_activation_function_output(activation_func)
            print "[%s] Files unavailable. Initializing empty neural network." \
                  % self.actor_file

        # Set up critic network
        self.critic = Ensemble(self.ensemble_size)
        if self.critic.create_from_file(self.critic_file):
            print "Critic Neural network loaded from %s" % self.actor_file
        else:
            # Loading failed, create a new Critic network
            self.critic = Ensemble(self.ensemble_size)
            self.progress = 0
            layers = [num_inputs, self.num_hiddens, 1]
            self.critic.create_standard_array(layers)
            self.critic.randomize_weights(-self.random_range, self.random_range)
            self.critic.set_activation_function_hidden(activation_func)
            self.critic.set_activation_function_output(libfann.LINEAR)
            print "[%s] Files unavailable. Initializing empty neural network." \
                  % self.critic_file

        ## Update the sigma and epsilon values to the correct value, assuming
        ## #progress iterations have already been performed.
        self.sigma = max(self.min_sigma, self.sigma * (self.random_decay ** self.progress))
        self.epsilon = max(self.min_epsilon, self.epsilon * (self.random_decay ** self.progress))
        self.alpha = max(self.min_alpha, self.alpha * (self.learning_decay ** self.progress))
        self.beta = max(self.min_beta, self.beta * (self.learning_decay ** self.progress))

        # Set correct learning rates
        self.actor.set_learning_rate(self.beta)
        self.critic.set_learning_rate(self.alpha)

        self.num_inputs = self.actor.get_num_input()

        if self.actor.get_num_input() != self.critic.get_num_input():
            raise Exception("Inputs of actor and critic networks do not match")
        if self.num_inputs != num_inputs:
            raise Exception("Wrong number of inputs: %d" % self.num_inputs)
        if self.num_outputs != self.actor.get_num_output():
            raise Exception("Wrong number of outputs: %d" % self.num_outputs)

    def save(self):
        """
        Save the Actor and Critic networks to a file. The location of the file
        has been specified to the constructor.
        """
        if self.test_performance:
            self.test_count += 1
            prog_file = os.path.basename(self.progress_file)
            prog_dir = os.path.dirname(self.progress_file)
            prog_file = os.path.join(prog_dir, "test_" + prog_file)

            f = open(prog_file, "w")
            f.write("%d" % self.test_count)
            f.close()
            return

        self.actor.save(self.actor_file)
        self.critic.save(self.critic_file)
        print "Saved actor to %s" % self.actor_file
        print "Saved critic to %s" % self.critic_file 

        self.sigma = max(self.min_sigma, self.sigma * self.random_decay)
        self.epsilon = max(self.min_epsilon, self.epsilon * self.random_decay)

        self.alpha = self.alpha * self.learning_decay
        self.beta = self.beta * self.learning_decay

        self.critic.set_learning_rate(self.alpha)
        self.actor.set_learning_rate(self.beta)
        print "Set learning rates to %f and %f" % (self.alpha, self.beta)

        self.progress += 1

        f = open(self.progress_file, "w")
        f.write("%d" % self.progress)
        f.close()

    def exploration(self, estimate):
        """
        Apply exploration to the estimate provided by the actor. Two exploratory
        strategies are supported:

        EXPLORE_GREEDY will select the estimate with probability 1 - epsilon,
        while it will select a random action with probability epsilon. 
    
        EXPLORE_GAUSSIAN will always explore: it will take a random sample out
        of the multivariate normal distribution with the mean at the estimate
        provided by the actor and the covariance equal for all variables and set
        to the epsilon variable.
        """
        if self.explore_strategy == EXPLORE_GREEDY:
            if self.test_performance:
                explore = numpy.random.random(1)[0] <= self.min_epsilon
            else:
                explore = numpy.random.random(1)[0] <= self.epsilon
            if explore:
                ran_values = numpy.random.random(len(estimate))
                result = [scale.scale(value, 0.0, 1.0, -1.0, 1.0) for value in ran_values]
            else:
                result = estimate
        elif self.explore_strategy == EXPLORE_GAUSSIAN:
            if self.test_performance:
                variances = [(self.min_sigma ** 2)] * len(estimate)
            else:
                variances = [(self.sigma ** 2)] * len(estimate)
            cov = numpy.diag(variances)

            # Draw random sample from Gaussian distribution
            result = numpy.random.multivariate_normal(estimate, cov)

            # Make sure all values are in the interval [-1, 1]
            result = [max(min(1.0, x), -1.0) for x in result]

        return result

    def update_networks(self, current_value, reward, terminal):
        """
        Update the Actor and the Critic networks, by comparing the current state
        value to the previous state value. If the state has increased, the
        action will be reinforced. The new value of the state is also calculated
        and updated in the critic network.
        """
        if not self.last_output or not self.last_value:
            print "[CACLA] Skipping first step - no previous step"
            return

        if self.test_performance:
            # Do not train network when testing performance
            return

        # Train Critic
        # Determine TD-error (formula 2)
        if terminal:
            print "Using terminal TD rule"
            td_error = reward - self.last_value
        else:
            td_error = reward + self.discount_factor * current_value \
                       - self.last_value

        if False:
            # This is the update rule, specified by the algorithm. 
            new_value = self.last_value + self.alpha * td_error
        else:
            # ANN implementation: just calculate the new value and train the
            # ANN on that. Alpha will be set as learning rate so it is still
            # used.
            new_value = self.last_value + td_error

        self.critic.train(self.last_state, [new_value])

        # Update running variance of the TD-error when using CACLA+Var
        if self.var_beta > 0:
            self.td_var = (1 - self.var_beta) * self.td_var + \
                          self.var_beta * (td_error ** 2)

        # Train Actor if state value improved as a result of performing the action
        # performed in the last state.
        if td_error > 0:
            updates = 1
            if self.var_beta > 0:
                updates = int(math.ceil(td_error / math.sqrt(self.td_var)))
                
            for i in range(updates):
                self.actor.train(self.last_state, self.last_output)

    def run(self, state, reward, terminal=False):
        """
        Run the CACLA algorithm, given the current state and the reward obtained
        in the current state. It will return a list of outputs, each value in
        the interval [-1, 1].

        The state should be a list; the length should be the same every time
        this function is called. If the networks should be loaded from a file,
        the length of the state descriptor should be equal to the length of the
        state descriptor used when the networks were originally created. If this
        is not the case, an exception will be thrown.

        The reward should be a value representing how good or bad the state is.
        It should be between (-max_reward, max_reward) where the value for
        max_reward can be obtained by calling the max_reward method. Only in
        exceptional cases, such as the goal state, this value can be exceeded.
        """
        # Check if the ANNs have been set up, and if not, do so.
        if not hasattr(self, 'actor'):
            self.setup_ann(state)

        # Run the ANNs
        nn_output = self.actor.run(state)
        value = self.critic.run(state)[0]

        # Update actor and critic
        self.update_networks(value, reward, terminal)
 
        # Apply exploration policy
        result = self.exploration(nn_output)

        # Store output for evaluation in the next iteration
        self.last_output = result[:]
        self.last_value = value
        self.last_state = state[:]
        self.total_reward += (self.discount_factor ** self.steps) *  reward

        self.steps += 1

        # We've got the next action, return it
        return result
