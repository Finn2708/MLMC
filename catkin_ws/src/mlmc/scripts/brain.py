import rospy
import matplotlib.pyplot as plt

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from typing import List, Tuple


def main():
    # Data storage objects
    set_speeds: List[Tuple[float, float]] = [(0., 0.)]
    enc_speeds: List[Tuple[float, float]] = [(0., 0.)]

    # Setup the node
    rospy.init_node("brain")

    # Setup service client
    test_run_trigger = rospy.ServiceProxy("test_run_trigger", Trigger)


    # Define subscriber callbacks
    def set_speed_cb(msg: Float32) -> None:
        set_speeds.append((rospy.get_time(), msg.data))


    def enc_speed_cb(msg: Float32) -> None:
        enc_speeds.append((rospy.get_time(), msg.data))


    while not rospy.is_shutdown():

        # Register subscribers
        sub_set_speed = rospy.Subscriber("setSpeed", Float32, set_speed_cb)
        sub_enc_speed = rospy.Subscriber("encoderSpeed", Float32, enc_speed_cb)

        # Reset data containers
        set_speeds = [(rospy.get_time(), 0.0)]
        enc_speeds = [(rospy.get_time(), 0.0)]

        # Start test run
        test_run_trigger()

        # Unregister subscribers
        sub_set_speed.unregister()
        sub_enc_speed.unregister()

        set_x = [t[0] - set_speeds[0][0] for t in set_speeds]
        set_y = [t[1] for t in set_speeds]

        enc_x = [t[0] - enc_speeds[0][0] for t in enc_speeds]
        enc_y = [t[1] for t in enc_speeds]

        plt.plot(enc_x, enc_y)
        plt.plot(set_x, set_y)
        plt.show()

        break


if __name__ == "__main__":
    main()
