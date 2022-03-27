import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from dataclasses import dataclass, field
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from mlmc_msgs.msg import PID
from typing import List, Tuple


@dataclass(init=True, repr=True, eq=True)
class PIDContainer:
    p: float = field(init=True, repr=True, default=0.1)
    i: float = field(init=True, repr=True, default=5.0)
    d: float = field(init=True, repr=True, default=0.0)
    ffd0: float = field(init=True, repr=True, default=0.0)
    ffd1: float = field(init=True, repr=True, default=0.0)


def squared_error(target: pd.Series, actual: pd.Series) -> pd.Series:
    err_data = []
    time_stamp = []
    for tar_idx, tar_data in zip(target.index, target):
        # Find the next matching index in actual
        act_idx_above = actual.index.get_indexer([tar_idx], method="bfill")[0]
        act_data = actual.iloc[act_idx_above]

        # Calculate mean squared error sum
        err_data.append((tar_data - act_data)**2)
        time_stamp.append(tar_idx)
    return pd.Series(data=err_data, index=time_stamp)


def main():
    # Data storage objects
    set_speeds: List[Tuple[float, float]] = [(0., 0.)]
    enc_speeds: List[Tuple[float, float]] = [(0., 0.)]

    # Setup the PID publisher
    pid_pub = rospy.Publisher("setPID", PID, queue_size=1)
    pid = PIDContainer()

    # Setup service client
    test_run_trigger = rospy.ServiceProxy("test_run_trigger", Trigger)

    # Setup the node
    rospy.init_node("brain")


    # Define subscriber callbacks
    def set_speed_cb(msg: Float32) -> None:
        set_speeds.append((rospy.get_time(), msg.data))


    def enc_speed_cb(msg: Float32) -> None:
        enc_speeds.append((rospy.get_time(), msg.data))


    while not rospy.is_shutdown():
        # Set new PIDs
        # HERE

        # Update MC
        pid_pub.publish(p=pid.p, i=pid.i, d=pid.d, ffd0=pid.ffd0, ffd1=pid.ffd1)

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

        # Create time series
        ts_set = pd.Series(data=[t[1] for t in set_speeds], index=[t[0] - set_speeds[0][0] for t in set_speeds])
        ts_enc = pd.Series(data=[t[1] for t in enc_speeds], index=[t[0] - set_speeds[0][0] for t in enc_speeds])
        
        ts_se = squared_error(ts_set, ts_enc)
        mse = ts_se.sum() / ts_se.size
        print(mse)

        # Show the data
        fig, ax1 = plt.subplots()
        fig.set_size_inches(12, 8)
        ax2 = ax1.twinx()
        ts_se.plot(ax=ax2, style="r", linewidth=1)
        ts_enc.plot(ax=ax1, linewidth=1)
        ts_set.plot(ax=ax1, style="k--", linewidth=1)


        def align_yaxis(ax1, ax2):
            """Align multiple y axis.
            
            Source:
            https://stackoverflow.com/a/54355867"""
            y_lims = np.array([ax.get_ylim() for ax in [ax1, ax2]])

            # force 0 to appear on both axes, comment if don't need
            y_lims[:, 0] = y_lims[:, 0].clip(None, 0)
            y_lims[:, 1] = y_lims[:, 1].clip(0, None)

            # normalize both axes
            y_mags = (y_lims[:,1] - y_lims[:,0]).reshape(len(y_lims),1)
            y_lims_normalized = y_lims / y_mags

            # find combined range
            y_new_lims_normalized = np.array([np.min(y_lims_normalized), np.max(y_lims_normalized)])

            # denormalize combined range to get new axes
            new_lim1, new_lim2 = y_new_lims_normalized * y_mags
            ax1.set_ylim(new_lim1)
            ax2.set_ylim(new_lim2)


        align_yaxis(ax1, ax2)

        plt.title(f"P={pid.p} I={pid.i} D={pid.d}\nFFD0={pid.ffd0} FFD1={pid.ffd1}")

        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Motor speed [Encoderticks / s]")
        ax2.set_ylabel("Squared Error")

        ax1.legend(["encoderSpeed", "setSpeed"], loc="upper left")
        ax2.legend(["Squared Error"], loc="upper right")

        fig.set_tight_layout(True)
        plt.show()

        break


if __name__ == "__main__":
    main()
