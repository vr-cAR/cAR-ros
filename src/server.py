import rospy
import sys
import importlib
from publisher import RosPublisher
import time 

def resolve_message_name(name, extension="msg"):
    try:
        names = name.split("/")
        module_name = names[0]
        class_name = names[1]
        importlib.import_module(module_name + "." + extension)
        module = sys.modules[module_name]
        if module is None:
            print("Failed to resolve module {}".format(module_name))
        module = getattr(module, extension)
        if module is None:
            print(
                "Failed to resolve module {}.{}".format(module_name, extension)
            )
        module = getattr(module, class_name)
        if module is None:
            print(
                "Failed to resolve module {}.{}.{}".format(module_name, extension, class_name)
            )
        return module
    except (IndexError, KeyError, AttributeError, ImportError) as e:
        print("Failed to resolve message name: {}".format(e))
        return None

def main():
    rospy.init_node("controller_node")
    module = resolve_message_name("std_msgs/Float32MultiArray")
    publisher = RosPublisher("ackermann_curvature_drive_fake", module)

    # First index is velocity, second is curvature
    while True:
        print("publishing")
        drive_data = [0.5, 0]
        publisher.send(drive_data)
        time.sleep(1)
    publisher.unregister()

if __name__ == "__main__":
    main()
