import rospy
from std_msgs.msg import Float32MultiArray 
import importlib
import sys

class RosPublisher():
    """
    Class to publish messages to a ROS topic
    """

    def __init__(self, topic, message_class, queue_size=10, latch=False):
        """
        Args:
            topic:         Topic name to publish messages to
            message_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        self.msg = message_class()
        self.pub = rospy.Publisher(topic, message_class, queue_size=queue_size, latch=latch)

    def send(self, data):
        """
        Takes in serialized message data from source outside of the ROS network,
        deserializes it into it's message class, and publishes the message to ROS topic.

        Args:
            data: The already serialized message_class data coming from outside of ROS
        """
        msg = Float32MultiArray()
        msg.data = [data[0], data[1]]
        #print(self.posmsg.velocity, ",", self.posmsg.curvature)
        self.pub.publish(msg)

    def unregister(self):
        self.pub.unregister()
