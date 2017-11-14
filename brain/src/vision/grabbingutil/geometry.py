import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __repr__(self):
        return "<Point (%d,%d)>" % (self.x, self.y)
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def as_cv_point(self):
        return (self.x, self.y)
    
    def distance(self, other):
        dx, dy = self.delta(other)
        return math.sqrt(dx * dx + dy * dy)

    def delta(self, other):
        return (
            abs(self.x - other.x),
            abs(self.y - other.y)
        )
    
    def angle_between(self, alpha, gamma):
        '''Returns the angle of corner self in line alpha-self-gamma.'''
        a = self.distance(gamma)
        c = alpha.distance(gamma)
        b = self.distance(alpha)
        # min(.., 1.0)?! Floating point math is a bitch
        return math.acos(min((a * a + b * b - c * c) / (2 * a * b), 1.0))
    
    def angle(self, other):
        '''Returns the angle as on a compass hold by self pointing to other.
        Specific for image coordinates (origin is top-left instead of
        bottom-left)
        Returns mathematical values:
            returns radians
            3 'o clock = 0 rad,
            couterclockwise = positive'''
        dx = other.x - self.x
        dy = self.y - other.y
        return math.atan2(dy, dx)
    
    def move(self, d):
        '''Returns a copy of this point moved by (x,y).'''
        return Point(self.x + int(d[0]), self.y + int(d[1]))

    @staticmethod
    def from_cv_point(pt):
        return Point(pt[0], pt[1])
    
class Rectangle:
    '''Class that represents a rectangle and contains some methods to do
    calculations on and with it. Does a lot of casting to ints.'''
    def __init__(self, x, y, width, height):
        self.x = int(round(x))
        self.y = int(round(y))
        self.width = int(round(width))
        self.height = int(round(height))
        
    def __repr__(self):
        '''Create something readable for your terminal'''
        return "<Rectangle x:%d y:%d width:%d height:%d>" % (self.x, self.y, self.width, self.height)
   
    @staticmethod
    def from_cv_rect(rect):
        '''Create a rectangle from a cv tuple'''
        assert len(rect) >= 4
        return Rectangle(rect[0], rect[1], rect[2], rect[3])
   
    def as_cv_rect(self):
        '''Return a tuple of 4 elements that cv functions will understand.'''
        return (self.x, self.y, self.width, self.height)
   
    @property
    def top_left(self):
        '''Tuple of x,y of the top-left corner of the rectangle.'''
        return (self.x, self.y)
    
    @property
    def bottom_right(self):
        '''Tuple of x,y of the bottom-right corner of the rectangle.'''
        return (self.x + self.width, self.y + self.height)
    
    @property
    def center(self):
        '''Tuple of x,y of the center of the rectangle.'''
        return (
            int(round(self.x + 0.5 * self.width)),
            int(round(self.y + 0.5 * self.height))
        )
        
    @property
    def size(self):
        '''Tuple of width,height of the rectangle.'''
        return (self.width, self.height)
    
    @property
    def is_real(self):
        '''Whether the rectangle can exist: are width and height positive?'''
        return self.width >= 0 and self.height >= 0
        
    def union(self, other):
        '''Returns the shared rectangle of two rectangles. If there is no
        overlap this may return negative values.'''
        top_left_x = max(self.top_left[0], other.top_left[0])
        top_left_y = max(self.top_left[1], other.top_left[1])
        bottom_right_x = min(self.bottom_right[0], other.bottom_right[0])
        bottom_right_y = min(self.bottom_right[1], other.bottom_right[1])
        
        return Rectangle(
            top_left_x, top_left_y,
            bottom_right_x - top_left_x, bottom_right_y - top_left_y)
    
    def contains(self, point):
        '''Returns true of the point tuple (x,y) lies inside the rectangle.'''
        return point[0] > self.x \
           and point[0] < self.x + self.width \
           and point[1] > self.y \
           and point[1] < self.y + self.height

    def move_to(self, x, y):
        '''Move this rectangle to x,y.'''
        self.x = int(x)
        self.y = int(y)


def space_between(left, right):
    '''returns a rectangle which contains the minimum space between
    left and right (arms!).'''
    x = left.x
    y = max(left.y, right.y)
    width = right.x + right.width - left.x
    height = min(left.y + left.height, right.y + right.height) - y
    return Rectangle(x, y, width, height)
