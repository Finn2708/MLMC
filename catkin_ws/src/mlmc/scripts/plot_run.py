import pandas
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from mlmc_msgs.msg import PID
from typing import List, Tuple


def plot_run(axs: plt.Axes ,ts_set: pandas.Series, ts_enc: pandas.Series, xlim: Tuple[float, float], ylim: Tuple[float, float], title: str="Last run") -> None:
    # Clear old plot
    axs.clear()

    # Plot setSpeed and encoderSpeed
    axs.plot(ts_set, linewidth=.8)
    axs.plot(ts_enc, linewidth=.8)

    # Format the plot
    axs.set_title(title)
    axs.set_xlim(xlim[0], xlim[1])
    axs.set_ylim(bottom=ylim[0], top=ylim[1])
    axs.grid(which="major")
    axs.set_xlabel("Time [s]")
    axs.set_ylabel("Speed [ticks / s]")
    axs.legend(["setSpeed", "encoderSpeed"])


def test_run(p: float, i: float, d: float) -> None:
    pid_pub = rospy.Publisher("setPID", PID, queue_size=1)

    rospy.wait_for_service("test_run_trigger")
    test_run_trigger = rospy.ServiceProxy("test_run_trigger", Trigger)

    rospy.init_node("test_run_plotter")
    rospy.sleep(3.0)

    # Set PIDs and wait for them to be active
    pid_pub.publish(p, i, d, 0., 0.)
    rospy.sleep(.5)

    set_speeds: List[float]
    enc_speeds: List[float]

    def set_speed_cb(msg: Float32) -> None:
        set_speeds.append((rospy.get_time(), msg.data))

    def enc_speed_cb(msg: Float32) -> None:
        enc_speeds.append((rospy.get_time(), msg.data))
    
    # Register subscribers
    sub_set_speed = rospy.Subscriber("setSpeed", Float32, set_speed_cb)
    sub_enc_speed = rospy.Subscriber("encoderSpeed", Float32, enc_speed_cb)

    # Reset data containers
    set_speeds = [(rospy.get_time(), 0.0)]
    enc_speeds = [(rospy.get_time(), 0.0)]

    try:
        test_run_trigger()
    except rospy.ServiceException as e:
        rospy.logwarn(f"Service call failure: {e}")
        return None

    ts_set = pandas.Series(data=[t[1] for t in set_speeds], index=[t[0] - set_speeds[0][0] for t in set_speeds])
    ts_enc = pandas.Series(data=[t[1] for t in enc_speeds], index=[t[0] - set_speeds[0][0] for t in enc_speeds])

    fig, ax = plt.subplots()
    
    plot_run(ax, ts_set, ts_enc, (-.5, 5.5), (-500, 2500), title="Baseline")

    plt.show()


def main():
    test_run(p=0.1, i=5.0, d=0.0)

if __name__ == "__main__":
    main()