"""
Utility module to control and configure various things in gazebo.

Copyright (C) 2015, Ron Snijders <ron@cno.nu>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy

from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

import gazebo_msgs.srv
import gazebo_msgs.msg
import std_srvs.srv

import tf
import math

#Modify this to the namespace of gazebo (rosservice list | grep gazebo):
gazebo_ns = "/gazebo"
service_timeout = 10

def move_to(model, x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Moves a model to the specified location and orientation.
    @param  model   The name of the model.
    """
    service = '%s/set_model_state' % gazebo_ns
    rospy.wait_for_service(service, timeout = service_timeout)
    set_model_state = rospy.ServiceProxy(service, gazebo_msgs.srv.SetModelState)
    msg = gazebo_msgs.msg.ModelState()
    msg.model_name = model
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]
    msg.reference_frame = "world"

    set_model_state(msg)

def get_position(name):
    """
    #param      The name of the model.
    @return     The position as (x, y, yaw).
    """
    service = '%s/get_model_state' % gazebo_ns
    rospy.wait_for_service(service, timeout = service_timeout)
    get_model_state = rospy.ServiceProxy(service, gazebo_msgs.srv.GetModelState)
    response = get_model_state(model_name = name)
    pos = {}
    #x and y:
    pos["x"] = response.pose.position.x
    pos["y"] = response.pose.position.y
    #yaw:
    quaternion = (
            response.pose.orientation.x,
            response.pose.orientation.y,
            response.pose.orientation.z,
            response.pose.orientation.w)
    pos["yaw"] = tf.transformations.euler_from_quaternion(quaternion)[2]
    return pos

def pause_physics():
    """
    Pauses physics (and thus the whole simulation).
    """
    service_name = '%s/pause_physics' % gazebo_ns
    rospy.wait_for_service(service_name, timeout = service_timeout)
    service = rospy.ServiceProxy(service_name, std_srvs.srv.Empty)
    service()

def unpause_physics():
    """
    Unpauses physics (and thus the whole simulation).
    """
    service_name = '%s/unpause_physics' % gazebo_ns
    rospy.wait_for_service(service_name, timeout = service_timeout)
    service = rospy.ServiceProxy(service_name, std_srvs.srv.Empty)
    service()

def delete_model(model_name):
    """
    Deletes the model with the specified name in gazebo.
    """
    rospy.wait_for_service('%s/delete_model' % gazebo_ns, timeout = service_timeout)
    service = rospy.ServiceProxy('%s/delete_model' % gazebo_ns, DeleteModel)
    service(model_name)

def insert_sdf(sdf_file, model_name, x, y, yaw, z = 0):
    """
    Inserts the SDF specified SDF file with the specified model name.
    """
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z

    quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    initial_pose.orientation.x = quat[0]
    initial_pose.orientation.y = quat[1]
    initial_pose.orientation.z = quat[2]
    initial_pose.orientation.w = quat[3]

    f = open(sdf_file, 'r')
    sdff = f.read()

    rospy.wait_for_service('%s/spawn_sdf_model' % gazebo_ns, timeout = service_timeout)
    service = rospy.ServiceProxy('%s/spawn_sdf_model' % gazebo_ns, SpawnModel)
    service(model_name, sdff, "%s_ns" % model_name, initial_pose, "world")
