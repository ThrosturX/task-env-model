#!/usr/bin/python
import argparse, traceback
import matplotlib.pyplot as plt
import numpy as np

def plot_task(task):
    profiles = task.get_profiles()
    for profile in profiles:
        plot_profile(profile)
        input("next?")

def plot_profile(profile):
    print(profile)
    high_energy, low_time = profile['min_time']
    plt.plot([low_time], [high_energy], 'ro')
    low_energy, high_time = profile['min_energy']
    plt.plot([high_time], [low_energy], 'bo')
    # some intermediate points
    times, energies = profile['curve']
    plt.plot(times, energies)
    # label the plot
    plt.xlabel('Time (s)')
    plt.ylabel('Energy (J)')
    plt.title('Tradeoff plot')
    plt.grid(True)
    plt.xlim(xmin=0)
    plt.ylim(ymin=0, ymax=profile['min_time'][0])
    plt.savefig('test.png')

def plot_results_on_profile(result_file, profile):
    plot_profile(profile)
    with open(result_file, 'r') as results:
        for result_line in results:
            if result_line[0] and result_line[0].isdigit():
                parts = [float(x) for x in result_line.split()]
                if parts[0] <= 0:
                    continue
                # format is STEPS, ENERGY, CLOCK, MIN_ENERGY
                plt.plot([parts[2]], [parts[1]], 'go')
                xmin, xmax = plt.xlim()
                ymin, ymax = plt.ylim()
                plt.xlim(xmax=max(xmax, parts[2]*1.05))
                plt.ylim(ymax=max(ymax, parts[1]*1.05))
    # now plot a line from min_e_t, min_e -> xmax, min_e
    # (the MINIMUM amount of energy doesn't depend on time)
    xmin, xmax = plt.xlim()
    ymin, ymax = plt.ylim()
    min_energy, m_e_t = profile['min_energy']
    plt.plot([m_e_t, xmax], [min_energy, min_energy], 'b')
    # now plot a line representing how much energy CAN be expended over time
    if 'power' in profile:
        power = profile['power']
        times = np.arange(0.0, xmax, 0.1)
        powers = [power * time for time in times]
        plt.plot(times, powers, 'r')
    plt.savefig('test.png')

def main(args):
    if args.filename:
#       task = samples.sample_system_1D_plotter(5000, default_start=20, default_delta=0)[0]
        task = samples.sample_system_1Db_plotter(default_start=20, default_delta=0)[0]
        profile = task.get_profile()
        profile['curve'] = task.calc_curve_time(0.01, profile['min_time'][1])
        plot_results_on_profile(args.filename, profile)


if __name__ == '__main__':
    import samples
    parser =  argparse.ArgumentParser(description="plot the power profile") 
    parser.add_argument("filename", metavar="FILE")
    args = parser.parse_args()
    try:
        main(args)
        plt.show()
    except SystemExit:
        exit(1)
    except KeyboardInterrupt:
        print('\rCTRL + C detected, canceling...')
        exit(2)
    except Exception as e:
        print('\nERROR')
        print('traceback:')
        print(traceback.print_exc())
        print('message:')
        print(e)

