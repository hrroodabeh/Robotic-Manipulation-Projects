#!/usr/bin/env python

# import numpy

import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF


def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    return t


# Our main class for computing Forward Kinematics


class ForwardKinematics(object):

    # Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        # Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)

    def callback(self, joint_values):
        # First, we must extract information about the kinematics of the robot from its URDF.
        # We will start at the root and add links and joints to lists
        link_name = self.robot.get_root()
        link_names = []
        joints = []
        while True:
            # Find the joint connected at the end of this link, or its "child"
            # Make sure this link has a child
            if link_name not in self.robot.child_map:
                break
            # Make sure it has a single child (we don't deal with forks)
            if len(self.robot.child_map[link_name]) != 1:
                rospy.logerror("Forked kinematic chain!");
                break
            # Get the name of the child joint, as well as the link it connects to
            (joint_name, next_link_name) = self.robot.child_map[link_name][0]
            # Get the actual joint based on its name
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                break;
            joints.append(self.robot.joint_map[joint_name])
            link_names.append(next_link_name)

            # Move to the next link
            link_name = next_link_name

        # Compute all the transforms based on the information we have
        all_transforms = self.compute_transforms(link_names, joints, joint_values)

        # Publish all the transforms
        self.pub_tf.publish(all_transforms)

    def compute_transforms(self, link_names, joints, joint_values):

        _parent = 'world_link'

        # a tf message, containing a list of TransformStamped objects
        all_transforms = tf.msg.tfMessage()

        # T is going to be used as a template transform matrix
        T = tf.transformations.identity_matrix()

        # transforms from each frame to the next is held in this list
        transform_chain = []

        # the name is quite clear:)
        world_to_link_transforms = []

        joint_names_m = []
        # we iterate through joints to get their specifications
        for joint in joints:
            joint_names_m.append(joint.name)

            # the translation of the link to joint at it's tip
            _translation = joint.origin.xyz

            # the rotation of joint at the end of the link
            rot_angle = tf.transformations.quaternion_from_euler(joint.origin.rpy[0],
                                                                 joint.origin.rpy[1],
                                                                 joint.origin.rpy[2])

            # the Transform matrix for the link to put joint in its position/orientation
            link_transform = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(_translation),
                tf.transformations.quaternion_matrix(rot_angle)
            )

            # if the joint rotates, we concatenate it with the previous link transform
            joint_angle = None
            if joint.type == 'revolute':
                rot_axis = joint.axis
                index = joint_values.name.index(joint.name)
                joint_angle = joint_values.position[index]

                if joint_angle != None:
                    joint_rotation = tf.transformations.quaternion_matrix(
                        tf.transformations.quaternion_about_axis(joint_angle, rot_axis)
                    )
                    link_transform = tf.transformations.concatenate_matrices(link_transform, joint_rotation)
            transform_chain.append(link_transform)

        t_temp = tf.transformations.identity_matrix()
        for i in range(len(transform_chain)):
            world_to_link_transforms.append(tf.transformations.concatenate_matrices(t_temp,
                                                                                    transform_chain[i]))

        for i in range(len(world_to_link_transforms)):
            all_transforms.transforms.append(convert_to_message(world_to_link_transforms[i],
                                                                link_names[i],
                                                                _parent))

        return all_transforms


if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()
