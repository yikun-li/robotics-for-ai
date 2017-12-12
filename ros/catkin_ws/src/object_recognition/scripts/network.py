import tensorflow as tf
import numpy as np
import cv2

class Network:
  def __init__(self):
    self.inputWidth = 32
    self.inputHeight = 32
    self.inputChannels = 3
    self.nLabels = 10
    self.learningRate = 0.002
    config = tf.ConfigProto()
    config.gpu_options.allow_growth=True
    self.sess = tf.InteractiveSession(config = config)
    self.build()
    self.saver = tf.train.Saver()
    
    
    
  def weight_variable(self, shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)

  def bias_variable(self, shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)

  def conv2d(self, x, W, padding = 'SAME'):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

  def max_pool_2x2(self, x, padding = 'SAME'):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding=padding)

  def store_checkpoint(self, ckpt):
      self.saver.save(self.sess, ckpt)
  def load_checkpoint(self, ckpt):
      self.saver.restore(self.sess, ckpt)

  def build(self):
    nKernelsL1 = 15
    kernelSizeL1 = 3
    
    nKernelsL2 = 25
    kernelSizeL2 = 12 # was 12
    FC1Size = 250
    
    #with tf.device('/gpu:0'):

    self.x = tf.placeholder(tf.float32, shape=[None, self.inputHeight, self.inputWidth, self.inputChannels])
    self.y_ = tf.placeholder(tf.float32, shape=[None, self.nLabels])

    image = tf.reshape(self.x, [-1, self.inputWidth, self.inputHeight, self.inputChannels])
    
    with tf.name_scope('conv1'):

      W1 = self.weight_variable([kernelSizeL1, kernelSizeL1, self.inputChannels, nKernelsL1])
      B1 = self.bias_variable([nKernelsL1])
      
      L1 = tf.nn.relu(self.conv2d(image, W1) + B1)
    
    with tf.name_scope('conv2'):
      W2 = self.weight_variable([kernelSizeL2, kernelSizeL2, nKernelsL1, nKernelsL2])
      B2 = self.bias_variable([nKernelsL2])
      
      L2 = tf.nn.relu(self.conv2d(L1, W2, 'VALID') + B2)
      
      L2 = tf.reshape(L2, [-1, self.inputWidth * self.inputHeight * nKernelsL2])
    
    with tf.name_scope('fc1'):
      W3 = self.weight_variable([self.inputWidth * self.inputHeight * nKernelsL2, FC1Size])
      B3 = self.bias_variable([FC1Size])
      
      self.L3 = tf.tanh(tf.matmul(L2, W3) + B3, name="sigmoid")
      
      
      
    with tf.name_scope('fc2'):
      W4 = self.weight_variable([FC1Size, self.nLabels])
      B4 = self.bias_variable([self.nLabels])
      
      self.L4 = tf.nn.softmax(tf.matmul(self.L3, W4) + B4, name="sigmoid2")

    
    cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(labels = self.y_, logits = self.L4), name="mean_cross_entropy")
    #self.train_step = tf.train.AdamOptimizer(self.learningRate).minimize(cross_entropy)
    self.train_step = tf.train.GradientDescentOptimizer(self.learningRate).minimize(cross_entropy)

    
    self.correct_prediction = tf.equal(tf.argmax(self.L4,1), tf.argmax(self.y_,1), name = "correct")
    self.accuracy = tf.reduce_mean(tf.cast(self.correct_prediction, tf.float32), name = "accuracy")
    
    self.summary = tf.summary.merge_all()
    print self.summary
    self.variables = tf.global_variables_initializer()
    print self.sess.run(self.variables)
    
    self.training_summary = tf.summary.scalar("training accuracy", self.accuracy)
    self.loss_summary = tf.summary.scalar("loss", cross_entropy)
    self.merged = tf.summary.merge_all()
    self.summary_writer = tf.summary.FileWriter("../tboard/", self.sess.graph)
  
  def train_batch(self, data, labels):
    #print "input shape@train: ", data.shape
    self.train_step.run(feed_dict = {self.x: data, self.y_: labels})
  
  def test_batch(self, data, labels):
    #print "input shape@test: ", data.shape
    return self.accuracy.eval(feed_dict = {self.x: data, self.y_: labels})
    
  
  def feed_batch(self, data):
    out = self.L4.eval(session= self.sess, feed_dict = {self.x: data})
    return np.argmax(out)
  
if __name__ == "__main__":
  network = Network()
