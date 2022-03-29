from cmath import inf
from shutil import which
from numpy import float32
import rospy
import random
import threading
import functools
import queue
import time
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from mlmc_msgs.msg import PID
from deap import base, creator, tools
from typing import Tuple, List

# Multithreaded Plotting copied from:
# https://stackoverflow.com/a/55268663

# Setup the ROS PID publisher
pid_pub = rospy.Publisher("setPID", PID, queue_size=1)

# Setup ROS service client
rospy.wait_for_service("test_run_trigger")
test_run_trigger = rospy.ServiceProxy("test_run_trigger", Trigger)

current_best = inf
POP_SIZE = 100
GEN_MAX = 20


#ript(Run In Plotting Thread) decorator
def ript(function):
    def ript_this(*args, **kwargs):
        global send_queue, return_queue, plot_thread
        if threading.currentThread() == plot_thread: #if called from the plotting thread -> execute
            return function(*args, **kwargs)
        else: #if called from a diffrent thread -> send function to queue
            send_queue.put(functools.partial(function, *args, **kwargs))
            return_parameters = return_queue.get(True) # blocking (wait for return value)
            return return_parameters
    return ript_this

#list functions in matplotlib you will use
functions_to_decorate = [[matplotlib.axes.Axes,'plot'],
                         [matplotlib.figure.Figure,'savefig'],
                         [matplotlib.backends.backend_tkagg.FigureCanvasTkAgg,'draw'],
                         ]
#add the decorator to the functions
for function in functions_to_decorate:
    setattr(function[0], function[1], ript(getattr(function[0], function[1])))


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


def evaluate_individual(individual: List[float]) -> Tuple[float,]:
    # Define subscriber callbacks
    def set_speed_cb(msg: Float32) -> None:
        set_speeds.append((rospy.get_time(), msg.data))


    def enc_speed_cb(msg: Float32) -> None:
        enc_speeds.append((rospy.get_time(), msg.data))

    # Set new PIDs
    print(individual)

    # Send PIDs to motor controller
    pid_pub.publish(p=individual[0], 
                    i=individual[1], 
                    d=individual[2], 
                    ffd0=0.0, 
                    ffd1=0.0)

    # Register subscribers
    sub_set_speed = rospy.Subscriber("setSpeed", Float32, set_speed_cb)
    sub_enc_speed = rospy.Subscriber("encoderSpeed", Float32, enc_speed_cb)

    # Reset data containers
    set_speeds = [(rospy.get_time(), 0.0)]
    enc_speeds = [(rospy.get_time(), 0.0)]

    try:
        # Trigger a test run
        test_run_trigger()
    except rospy.ServiceException as e:
        rospy.logwarn(f"Service call failure: {e}")
        return (300000,)


    # Unregister subscribers
    sub_set_speed.unregister()
    sub_enc_speed.unregister()

    # Send known good PIDs to motor controller
    pid_pub.publish(p=.1, 
                    i=5., 
                    d=0., 
                    ffd0=0., 
                    ffd1=0.)

    # And wait for the motor to recover from (possibly) terrible PIDs
    rospy.sleep(.5)

    # Create time series
    ts_set = pd.Series(data=[t[1] for t in set_speeds], index=[t[0] - set_speeds[0][0] for t in set_speeds])
    ts_enc = pd.Series(data=[t[1] for t in enc_speeds], index=[t[0] - set_speeds[0][0] for t in enc_speeds])

    global ax, fig
    xlim_min, xlim_max = (-.5, 5.5)

    plt_ax = ax[0, 0]
    plt_ax.clear()

    plt_ax.plot(ts_set, linewidth=.8)
    plt_ax.plot(ts_enc, linewidth=.8)

    plt_ax.set_title("Last individual")
    plt_ax.set_xlim(xlim_min, xlim_max)
    plt_ax.set_ylim(bottom=-500., top=2500.)
    plt_ax.grid(which="major")
    plt_ax.set_xlabel("Time [s]")
    plt_ax.set_ylabel("Speed [ticks / s]")
    plt_ax.legend(["setSpeed", "encoderSpeed"])
    
    ts_se = squared_error(ts_set, ts_enc)

    err_ax = ax[0, 1]
    err_ax.clear()

    err_ax.plot(ts_se, linewidth=.6, color="red")

    err_ax.set_title("Last Squared Error")
    err_ax.set_xlim(xlim_min, xlim_max)
    err_ax.set_ylim(0, 1000000)
    err_ax.grid(which="major")
    err_ax.set_xlabel("Time [s]")
    err_ax.set_ylabel("Squared Error [ticks² / s²]")


    mse = ts_se.sum() / ts_se.size

    global current_best

    if mse < current_best:
        current_best = mse
        best_ax = ax[1, 0]    
        best_ax.clear()

        best_ax.plot(ts_set, linewidth=.8)
        best_ax.plot(ts_enc, linewidth=.8)

        best_ax.set_xlim(xlim_min, xlim_max)
        best_ax.set_ylim(bottom=-500., top=2500.)
        best_ax.grid(which="major")
        best_ax.set_title("Best individual")
        best_ax.set_xlabel("Time [s]")
        best_ax.set_ylabel("Speed [ticks / s]")
        best_ax.legend(["setSpeed", "encoderSpeed"])

    fig.canvas.draw()

    return (mse, )


def setup_ga() -> base.Toolbox:
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    # Initial genes are generated randomly
    toolbox.register("attr_float", random.uniform,  0., 5.)
    # Each individual should have 3 attr_floats
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, 3)
    # Population should be a number of individuals
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)


    # Evaluate
    toolbox.register("evaluate", evaluate_individual)
    # Crossover
    toolbox.register("mate", tools.cxTwoPoint)
    # Mutation (rate=0.05)
    toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
    # Breeding select
    toolbox.register("select", tools.selTournament, tournsize=3)

    return toolbox


# function that checks the send_queue and executes any functions found
def update_figure(window, send_queue, return_queue):
    try:
        callback = send_queue.get(False)  # get function from queue, false=doesn't block
        return_parameters = callback() # run function from queue
        return_queue.put(return_parameters)
    except:
        None
    window.after(10, update_figure, window, send_queue, return_queue)


def plot():
    # we use these global variables because we need to access them from within the decorator
    global plot_thread, send_queue, return_queue
    return_queue = queue.Queue()
    send_queue = queue.Queue()
    plot_thread=threading.currentThread()
    # we use these global variables because we need to access them from the main thread
    global ax, fig
    fig, ax = plt.subplots(2, 2)
    fig.set_size_inches(12, 8)
    fig.set_tight_layout(True)
    # we need the matplotlib window in order to access the main loop
    window=plt.get_current_fig_manager().window
    # we use window.after to check the queue periodically
    window.after(10, update_figure, window, send_queue, return_queue)
    # we start the main loop with plt.plot()
    plt.show()


def main():
    #start the plot and open the window
    thread = threading.Thread(target=plot)
    thread.setDaemon(True)
    thread.start()
    time.sleep(1) #we need the other thread to set 'fig' and 'ax' before we continue

    # Reproducibility
    random.seed(123)

    # Setup the node
    rospy.init_node("brain")
    rospy.sleep(3.0)  # Give time for setup

    # Setup the genetic algorithm
    toolbox = setup_ga()

    pop = toolbox.population(n=POP_SIZE)
    CXPB, MUTPB = 0.5, 0.2

    fitnesses = list(map(toolbox.evaluate, pop))

    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit

    print(f"   Evaluated {len(pop)} individuals")

    fits = [ind.fitness.values[0] for ind in pop]
    
    length = len(pop)
    mean = sum(fits) / length
    sum2 = sum(x * x for x in fits)
    std = abs(sum2 / length - mean**2)**.5

    gen_counter = 0

    global ax, fig
    gen = [0]
    min_fitness = [max(fit)]
    avg_fitness = [mean]

    while not rospy.is_shutdown():
        while max(fits) > 10000 and gen_counter < GEN_MAX:
            gen_counter += 1
            gen.append(gen_counter)

            print(f"-- Generation {gen_counter} --")

            # Select the next gen individuals
            offspring = toolbox.select(pop, len(pop))
            # Clone the selected individuals
            offspring = list(map(toolbox.clone, offspring))

            # Apply crossover and mutation on the offspring
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                # Cross two individuals with probability CXPB
                if random.random() < CXPB:
                    toolbox.mate(child1, child2)
                    # Fitness has to be calculated later on
                    del child1.fitness.values
                    del child2.fitness.values

            for mutant in offspring:
                # Mutate an individual with probability MUTPB
                if random.random() < MUTPB:
                    toolbox.mutate(mutant)
                    del mutant.fitness.values

            # Evaluate the individuals with an invalid fitness
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = map(toolbox.evaluate, invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit
            
            print(f"   Evaluated {len(invalid_ind)} individuals")

            pop[:] = offspring
            fits = [ind.fitness.values[0] for ind in pop]

            length = len(pop)
            mean = sum(fits) / length
            sum2 = sum(x * x for x in fits)
            std = abs(sum2 / length - mean**2)**.5

            print(f"   Min {min(fits)}")
            print(f"   Max {max(fits)}")
            print(f"   Avg {mean}")
            print(f"   Std {std}")

            min_fitness.append(min(fits))
            avg_fitness.append(mean)

            stats_ax = ax[1, 1]
            stats_ax.clear()

            stats_ax.plot(gen, min_fitness)
            stats_ax.plot(gen, avg_fitness)

            stats_ax.grid(which="major")

            stats_ax.set_title("Fitness")
            stats_ax.set_xlabel("Generation")
            stats_ax.set_ylabel("Fitness")
            stats_ax.legend(["Minimum", "Average"])
            fig.canvas.draw()

        print("-- End of (successful) evolution --")
        best_ind = tools.selBest(pop, 1)[0]
        print(f"Best individual is {best_ind}, {best_ind.fitness.values}")

        break

    # End drawing thread
    thread.join()


if __name__ == "__main__":
    main()
