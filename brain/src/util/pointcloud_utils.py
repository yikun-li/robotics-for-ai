import body.bodycontroller
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import numpy as np
import tf
import os
import time 

listener = tf.TransformListener()

def pointcloud2_to_array(cloud_msg): 
    ''' Converts a rospy PointCloud2 message to a numpy recordarray
        Assumes all fields 32 bit floats, and there is no padding. 
        
        THIS FUNCTION IS RECOMMENDED '''
        
    dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]   # Get data types
    dtype_list.append(('rgb', np.float32))
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)   # Fill np array with data
    return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))
    
def get_xyz_points(cloud_array, remove_nans=True): 
    ''' Pulls out x, y, and z columns from the cloud recordarray, and returns a 3xN matrix. ''' 

    # remove crap points 
    if remove_nans: 
        mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z']) 
        cloud_array = cloud_array[mask] 

    # pull out x, y, and z values 
    points = np.zeros(list(cloud_array.shape) + [3], dtype=np.float) 
    points[...,0] = cloud_array['x']    # Remember that x = 0 is inside the robot. Subtract 0.2 from x to get actual distance
    points[...,1] = cloud_array['y'] 
    points[...,2] = cloud_array['z']

    return points 
   
def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    ''' Converts a PointCloud2 message to a 3xN matrix [x,y,z] x N '''
    
    return get_xyz_points(pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)
    
    
def transform_point_using_service(point, frame_id, target_frame_id):
    ''' Transforms point to target frame id '''

    rospy.wait_for_service('transform_point')
    try:
        tf_point_server = rospy.ServiceProxy('transform_point', TransformPoint)
        resp = tf_point_server(point['x'], point['y'], point['z'], frame_id, target_frame_id)
        return {'x': resp.t_x, 'y': resp.t_y, 'z': resp.t_z}
    except rospy.ServiceException, e:
        print "[TransformPoint] Transform failed"
        return {}
    
    
def transform_point(point, frame_id, target_frame_id, stamp = rospy.Time(0), persist = True):
    ''' Transforms point to target frame id '''
    
    # Check arguments
    if not 'x' in point or not 'y' in point or not 'z' in point:
        print "[TransformPoint] Incorrect point given"
        return {}
    
    # Fail saves
    max_tries = 3
    current_try = 1
    transform_found = False
    moved_robot = False
    bc = body.bodycontroller.BodyController()
    
    # Check if transformation is possible
    while current_try <= max_tries:
        try:
            listener.waitForTransform(frame_id, target_frame_id, stamp, rospy.Duration(1.0))
            transform_found = True
            break
        except:
            print "[TransformPoint] Could not find transform between %s and %s" % (frame_id, target_frame_id)
        # Check tries
        if current_try < max_tries:
            print "[TransformPoint] Will try again"
        else:
            print "[TransformPoint] Lookup transform failed for " + str(max_tries) + " tries"
            if persist and not moved_robot:
                print "[TransformPoint] I will try to move a bit"
                bc.pioneer(0).forward(100)
                time.sleep(2)
                bc.pioneer(0).forward(-100)
                time.sleep(2)
                moved_robot = True
                current_try = 0
            else:
                print "[TransformPoint] Giving up"
        current_try += 1
            
    if not transform_found:
        return {}
        
    # Create PointStamped message
    point_stamped = PointStamped()
    point_stamped.header.stamp = stamp
    point_stamped.header.frame_id = frame_id
    point_stamped.point.x = point['x']
    point_stamped.point.y = point['y']
    point_stamped.point.z = point['z']
    
    # Transform point
    point_stamped_tf = listener.transformPoint(target_frame_id, point_stamped)
    
    # Return point
    return {'x': point_stamped_tf.point.x, 'y': point_stamped_tf.point.y, 'z': point_stamped_tf.point.z}
    

def is_point_inside_map(fileName, point, frame_id):
    ''' Checks if a given point is inside the map boundaries '''

    # Retrieve the boundary points
    boundaryPoints = readArenaBoundaries(os.environ['BORG'] + fileName)

    # Check if file was readable
    if not boundaryPoints or len(boundaryPoints) < 3:
        print "Can't check if point is inside map"
        return False

    # Transform point, if necessary
    point = transform_point(point, frame_id, "/map")
    
    # Check if point is inside map
    n = len(boundaryPoints)
    inside = False
    x1,y1 = boundaryPoints[0]
    for i in range(n+1):
        x2,y2 = boundaryPoints[i % n]
        if point['y'] >= min(y1,y2):
            if point['y'] <= max(y1,y2):
                if point['x'] <= max(x1,x2):
                    if y1 != y2:
                        xinters = (point['y']-y1)*(x2-x1)/(y2-y1)+x1
                    if x1 == x2 or point['x'] < xinters:
                        inside = not inside
        x1,y1 = x2,y2
    return inside     
    
def readArenaBoundaries(fileName):
    ''' Reads location file that contains the arena boundaries. 
        Stores the x,y coordinates of the corners in a list of tuples. '''
    try:
        fileHandle = open(fileName, 'r')
    except:
        print "Cannot open location file " + fileName
        return False  
    firstLineSkipped = False
    boundaryPoints = []
    for line in fileHandle.readlines():
        line = line.rstrip("\n")    # remove endline
        # Skip first line with keys
        if not firstLineSkipped:
            firstLineSkipped = True
            continue
        # Parse line
        values = line.split(',')  
        # Skip when line does not contain 4 arguments
        if len(values) != 4:
            continue   
        # Add boundary
        boundaryPoints.append((float(values[1]), float(values[2]))) 
    return boundaryPoints





    
