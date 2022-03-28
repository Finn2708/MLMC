#!/usr/bin/env python

from os import pread
from tracemalloc import start
import rospy
import numpy as np
import scipy.signal
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


def square_wave_gen(runtime: float=30., period: float=10, max_speed: float=2000., duty: float=0.5) -> float:
    t = np.linspace(0, runtime, 1000)
    s = (scipy.signal.square(2 * np.pi * t * 1 / period, duty) + 1) / 2 * max_speed

    start_time = rospy.get_time()
    while rospy.get_time() - start_time < runtime:
        current_time = rospy.get_time() - start_time
        idx = np.searchsorted(t, current_time)
        yield s[idx]
    yield 0.0


def sawtooth_wave_gen(runtime: float=30., period: float=10, max_speed: float=2000., width: float=1.) -> float:
    t = np.linspace(0, runtime, 1000)
    s = (scipy.signal.sawtooth(2 * np.pi * t * 1 / period, width) + 1) / 2 * max_speed

    start_time = rospy.get_time()
    while rospy.get_time() - start_time < runtime:
        current_time = rospy.get_time() - start_time
        idx = np.searchsorted(t, current_time)
        yield s[idx]
    yield 0.0


def chirp_wave_gen(runtime: float=30., period: float=8, max_speed: float=2000., t1: float=30., period1: float=8) -> float:
    t = np.linspace(0, runtime, 1000)
    s = (scipy.signal.chirp(2 * np.pi * t * 1 / period, 1 / period, t1, 1 / period1, method="linear") + 1) / 2 * max_speed

    start_time = rospy.get_time()
    while rospy.get_time() - start_time < runtime:
        current_time = rospy.get_time() - start_time
        idx = np.searchsorted(t, current_time)
        yield s[idx]
    yield 0.0


def handle_trigger(request: TriggerRequest) -> TriggerResponse:

    pub = rospy.Publisher("setSpeed", Float32, queue_size=10)
    msg = Float32()

    pub.publish(data=0.0)
    r = rospy.Rate(700)

    for speed in square_wave_gen(runtime=5.0, period=6.0):
        pub.publish(data=speed)
        r.sleep()
    
    return TriggerResponse(True, "Test done")


def test_run_server():
    rospy.init_node("test_run_server")
    s = rospy.Service("test_run_trigger", Trigger, handle_trigger)
    rospy.spin()


if __name__=="__main__":
    test_run_server()