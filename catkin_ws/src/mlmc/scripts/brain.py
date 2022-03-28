import rospy
import random
import pandas as pd
import matplotlib.pyplot as plt

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from mlmc_msgs.msg import PID
from deap import base, creator, tools
from typing import Tuple, List


# Setup the ROS PID publisher
pid_pub = rospy.Publisher("setPID", PID, queue_size=1)

# Setup ROS service client
test_run_trigger = rospy.ServiceProxy("test_run_trigger", Trigger)


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


def evaluateIndividual(individual: List[float, float, float, float, float]) -> Tuple[float,]:
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
                    ffd0=individual[3], 
                    ffd1=individual[4])

    # Register subscribers
    sub_set_speed = rospy.Subscriber("setSpeed", Float32, set_speed_cb)
    sub_enc_speed = rospy.Subscriber("encoderSpeed", Float32, enc_speed_cb)

    # Reset data containers
    set_speeds = [(rospy.get_time(), 0.0)]
    enc_speeds = [(rospy.get_time(), 0.0)]

    # Trigger a test run
    test_run_trigger()

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
    
    ts_se = squared_error(ts_set, ts_enc)
    mse = ts_se.sum() / ts_se.size
    return (mse, )


def setup_ga() -> base.Toolbox:
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    # Initial genes are generated randomly
    toolbox.register("attr_float", random.uniform,  0., 1.)
    # Each individual should have 5 attr_floats
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, 5)
    # Population should be a number of individuals
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)


    # Evaluate
    toolbox.register("evaluate", evaluateIndividual)
    # Crossover
    toolbox.register("mate", tools.cxTwoPoint)
    # Mutation (rate=0.05)
    toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
    # Breeding select
    toolbox.register("select", tools.selTournament, tournsize=3)

    return toolbox


def main():
    # Reproducibility
    random.seed(123)

    # Setup the node
    rospy.init_node("brain")

    # Setup the genetic algorithm
    toolbox = setup_ga()

    pop = toolbox.population(n=50)
    CXPB, MUTPB = 0.5, 0.2

    fitnesses = list(map(toolbox.evaluate, pop))

    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
    
    print(f"   Evaluated {len(pop)} individuals")

    fits = [ind.fitness.values[0] for ind in pop]

    gen = 0

    while not rospy.is_shutdown():
        while max(fits) > -10000 and gen < 20:
            gen += 1
            print(f"-- Generation {gen} --")

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

        print("-- End of (successful) evolution --")

        best_ind = tools.selBest(pop, 1)[0]

        print(f"Best individual is {best_ind}, {best_ind.fitness.values}")

        break


        """
        # Show the data
        fig, ax1 = plt.subplots()
        fig.set_size_inches(12, 8)
        ax2 = ax1.twinx()
        ts_se.plot(ax=ax2, style="r", linewidth=1)
        ts_enc.plot(ax=ax1, linewidth=1)
        ts_set.plot(ax=ax1, style="k--", linewidth=1)


        def align_yaxis(ax1, ax2):
        """ # Align multiple y axis.
        
        # Source:
        # https://stackoverflow.com/a/54355867
        """
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
        """


if __name__ == "__main__":
    main()
