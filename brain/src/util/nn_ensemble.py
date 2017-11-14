from pyfann import libfann
import types
import numpy
import random
import sys
import os

class Ensemble(object):
    def __init__(self, size = 2):
        if (size < 1):
            raise Exception("An ensemble must consist of at least 1 ANN")

        self.ANNs = []
        for i in range(size):
            self.ANNs.append(libfann.neural_net())

    def __getattr__(self, name):
        try:
            obj = object.__getattr__(name)
            return obj
        except:
            pass

        if hasattr(self.ANNs[0], name):
            obj = getattr(self.ANNs[0], name)
            if hasattr(obj, '__call__'):
                parent_obj = self
                def method(*args, **kwargs):
                    return parent_obj.__wrap_function(name, *args, **kwargs)
                return method
            else:
                return obj
        raise Exception("Method %s does not exist in neural_net" % name)

    def __wrap_function(self, name, *args, **kwargs):
        results = []
        for ANN in self.ANNs:
            func = getattr(ANN, name)
            results.append(func(*args, **kwargs))

        return self.__merge_results(results)

    def __merge_results(self, results):
        first = results[0]
        if type(first) is tuple or type(first) is list:
            return list(numpy.average(results, 0))
        elif type(first) is int or type(first) is float:
            return numpy.average(results)
        elif type(first) is None:
            return None
        else:
            return results

    def save(self, filename):
        base = os.path.basename(filename)
        ext_start= base.rfind('.')
        ext = base[ext_start:]
        base = base[:ext_start]
        path = os.path.dirname(filename)

        for idx, ANN in enumerate(self.ANNs):
            curfile = "%s_%03d%s" % (base, idx, ext)
            filepath = os.path.join(path, curfile)
            ANN.save(filepath)

    def create_from_file(self, filename):
        base = os.path.basename(filename)
        ext_start= base.rfind('.')
        ext = base[ext_start:]
        base = base[:ext_start]
        path = os.path.dirname(filename)

        for idx, ANN in enumerate(self.ANNs):
            curfile = "%s_%03d%s" % (base, idx, ext)
            filepath = os.path.join(path, curfile)
            if not ANN.create_from_file(filepath):
                return False
        return True

if __name__ == "__main__":
    num = 25
    if len(sys.argv) > 1:
        num = int(sys.argv[1])

    print "Using %d ANNs" % num
    a = Ensemble(num)
    layers = [2, 4, 1]
    a.create_standard_array(layers)
    a.set_learning_rate(0.6)
    a.set_activation_function_hidden(libfann.SIGMOID)
    a.set_activation_function_output(libfann.SIGMOID)

    data = [([0, 0], [0]),
            ([0, 1], [1]),
            ([1, 0], [1]),
            ([1, 1], [0])]
    tdata = libfann.training_data()
    tdata.read_train_from_file('xor.data')

    for cur_in, cur_out in data:
        res = a.run(cur_in)
        print "(%d, %d) -> %d" % (cur_in[0], cur_in[1], res[0])

    a.train_on_data(tdata, 20, 0, 0.0001)

    for cur_in, cur_out in data:
        res = a.run(cur_in)
        print "(%.2f, %.2f) -> %.2f" % (cur_in[0], cur_in[1], res[0])
