import stats
import math
import scipy.stats
import logging
import util.nullhandler

logging.getLogger('Borg.Brain.BBIE.IntervalEstimation').addHandler(util.nullhandler.NullHandler())

class IntervalEstimation(object):
    def __init__(self):
        self.logger = logging.getLogger('Borg.Brain.BBIE.IntervalEstimation')

    def get_upperbound( self, confidence_interval, lst ):
        return self.get_mean( lst ) + (self.get_z( confidence_interval ) * stats.stdev( lst ) / math.sqrt ( len( lst ) ) )
        
    def get_lowerbound( self, confidence_interval, lst ):
        return self.get_mean( lst ) - (self.get_z( confidence_interval ) * stats.stdev( lst ) / math.sqrt ( len( lst ) ) )
        
    def get_mean( self, lst ):
        return stats.mean( lst )
            
        
    def get_z( self, confidence_interval ):
        q = (100.0 - confidence_interval) / 200
        return scipy.stats.norm.ppf( 1 - q )    

