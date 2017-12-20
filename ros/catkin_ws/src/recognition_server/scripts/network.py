from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import tensorflow as tf


class Network:
    def __init__(self):
        self.inputSize = 32
        self.kernelSize = 5
        self.numberOfKernels = 32
        self.numberOfNeuron = 1024
        self.learningRate = 0.0001
        self.sess = tf.InteractiveSession()
        self.train_step = None
        self.accuracy = None
        self.x = None
        self.y_ = None
        self.build()

    def build(self):
        self.x = tf.placeholder(tf.float32, shape=[None, self.inputSize, self.inputSize, 3])
        self.y_ = tf.placeholder(tf.float32, shape=[None, 10])

        # First convolutional layer - maps one grayscale image to 32 feature maps.
        with tf.name_scope('conv1'):
            W_conv1 = self.weight_variable([self.kernelSize, self.kernelSize, 3, self.inputSize])
            b_conv1 = self.bias_variable([self.inputSize])
            h_conv1 = tf.nn.relu(self.conv2d(self.x, W_conv1) + b_conv1)

        # Pooling layer - downsamples by 2X.
        with tf.name_scope('pool1'):
            h_pool1 = self.max_pool_2x2(h_conv1)

        # Second convolutional layer -- maps 32 feature maps to 64.
        with tf.name_scope('conv2'):
            W_conv2 = self.weight_variable(
                [self.kernelSize, self.kernelSize, self.numberOfKernels, self.numberOfKernels * 2])
            b_conv2 = self.bias_variable([self.numberOfKernels * 2])
            h_conv2 = tf.nn.relu(self.conv2d(h_pool1, W_conv2) + b_conv2)

        # Second pooling layer.
        with tf.name_scope('pool2'):
            h_pool2 = self.max_pool_2x2(h_conv2)

        # Fully connected layer 1 -- after 2 round of downsampling, our 28x28 image
        # is down to 7x7x64 feature maps -- maps this to 1024 features.
        with tf.name_scope('fc1'):
            W_fc1 = self.weight_variable([int(self.inputSize / 4 * self.inputSize / 4 * self.numberOfKernels * 2),
                                          self.numberOfNeuron])
            b_fc1 = self.bias_variable([self.numberOfNeuron])

            h_pool2_flat = tf.reshape(h_pool2,
                                      [-1, int(self.inputSize / 4 * self.inputSize / 4 * self.numberOfKernels * 2)])
            h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

        # Dropout - controls the complexity of the model, prevents co-adaptation of
        # features.
        with tf.name_scope('dropout'):
            self.keep_prob = tf.placeholder(tf.float32)
            h_fc1_drop = tf.nn.dropout(h_fc1, self.keep_prob)

        # Map the 1024 features to 10 classes, one for each digit
        with tf.name_scope('fc2'):
            W_fc2 = self.weight_variable([1024, 10])
            b_fc2 = self.bias_variable([10])

            y_conv = tf.matmul(h_fc1_drop, W_fc2) + b_fc2

        with tf.name_scope('loss'):
            cross_entropy = tf.nn.softmax_cross_entropy_with_logits(labels=self.y_,
                                                                    logits=y_conv)
        cross_entropy = tf.reduce_mean(cross_entropy)

        with tf.name_scope('adam_optimizer'):
            self.train_step = tf.train.AdamOptimizer(self.learningRate).minimize(cross_entropy)

        with tf.name_scope('accuracy'):
            correct_prediction = tf.equal(tf.argmax(y_conv, 1), tf.argmax(self.y_, 1))
            correct_prediction = tf.cast(correct_prediction, tf.float32)
        self.accuracy = tf.reduce_mean(correct_prediction)
        self.sess.run(tf.global_variables_initializer())

    def store_checkpoint(self, file_name):
        saver = tf.train.Saver()
        save_path = saver.save(self.sess, file_name)
        print("Model saved in file: %s" % save_path)

    def restore_checkpoint(self):
        saver = tf.train.Saver()
        saver.restore(self.sess, "/tmp/model.ckpt")
        print("Model restored.")

    def train_batch(self, data, labels):
        self.train_step.run(session=self.sess, feed_dict={self.x: data, self.y_: labels, self.keep_prob: 0.5})

    def test_batch(self, data, labels):
        return self.accuracy.eval(session=self.sess, feed_dict={self.x: data, self.y_: labels, self.keep_prob: 1.0})

    # def feed_batch(self, data):
    #     out = self.L4.eval(session=self.sess, feed_dict={self.x: data})
    #     return np.argmax(out)

    def load_checkpoint(self, x):
        pass

    def conv2d(self, x, W):
        """conv2d returns a 2d convolution layer with full stride."""
        return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

    def max_pool_2x2(self, x):
        """max_pool_2x2 downsamples a feature map by 2X."""
        return tf.nn.max_pool(x, ksize=[1, 2, 2, 1],
                              strides=[1, 2, 2, 1], padding='SAME')

    def weight_variable(self, shape):
        """weight_variable generates a weight variable of a given shape."""
        initial = tf.truncated_normal(shape, stddev=0.1)
        return tf.Variable(initial)

    def bias_variable(self, shape):
        """bias_variable generates a bias variable of a given shape."""
        initial = tf.constant(0.1, shape=shape)
        return tf.Variable(initial)


if __name__ == '__main__':
    network = Network()
