#!/usr/bin/python
import argparse, traceback
import matplotlib.pyplot as plt
import numpy as np

def plot_task(task):
    profiles = task.get_profiles()
    for profile in profiles:
        plot_profile(profile)
        input("next?")

def plot_profile(profile, fname='default'):
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
    plt.savefig(fname.strip('.txt') + '.png')

def plot_results_on_profile(result_file, profile):
    # very green = early result
    # very dark green = late result
    # more red = more recent result
    def next_color(current):
        green_str = current[3:5]
        green = int(green_str, 16) - 3
        red_str = current[1:3]
        red = int(red_str, 16)
        blue = int(current[5:7])
        if green > 255 \
        or green < 0:
            if red == 0:
                red = 64
            green = 125
            red = red + 17
        if red >= 255:
            green = 0
            red = 255
            blue += 1
            if blue > 255:
                blue = 255
        color = '#{:02X}{:02X}{:02X}'.format(red, green, blue)
        return color
    plot_profile(profile, result_file)
    with open(result_file, 'r') as results:
        current_color = '#00FF00'
        for result_line in results:
            current_color = next_color(current_color)
            if result_line[0] and result_line[0].isdigit():
                parts = [float(x) for x in result_line.split()]
                if parts[0] <= 0:
                    continue
                # format is STEPS, ENERGY, CLOCK, MIN_ENERGY
                plt.plot([parts[2]], [parts[1]], 'x', color=current_color)
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
        task = samples.sample_system_1Db_plotter2(default_start=20, default_delta=0)[0]
        for var in task.all_objects():
            if hasattr(var, 'name'):
                print("{}: {}".format(var.name, var))
        profile = task.get_profile()
        profile['curve'] = task.calc_curve_time(0.01, profile['min_energy'][1])
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

