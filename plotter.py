#!/usr/bin/python
import argparse, traceback
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
#%matplotlib inline
from math import sqrt
SPINE_COLOR = 'gray'

def latexify(fig_width=None, fig_height=None, columns=1):
    """Set up matplotlib's RC params for LaTeX plotting.
    Call this before plotting a figure.

    Parameters
    ----------
    fig_width : float, optional, inches
    fig_height : float,  optional, inches
    columns : {1, 2}
    """

    # code adapted from http://www.scipy.org/Cookbook/Matplotlib/LaTeX_Examples

    # Width and max height in inches for IEEE journals taken from
    # computer.org/cms/Computer.org/Journal%20templates/transactions_art_guide.pdf

    assert(columns in [1,2])

    if fig_width is None:
        fig_width = 3.39 if columns==1 else 6.9 # width in inches

    if fig_height is None:
        golden_mean = (sqrt(5)-1.0)/2.0    # Aesthetic ratio
        fig_height = fig_width*golden_mean # height in inches

    MAX_HEIGHT_INCHES = 8.0
    if fig_height > MAX_HEIGHT_INCHES:
        print("WARNING: fig_height too large:" + fig_height + 
              "so will reduce to" + MAX_HEIGHT_INCHES + "inches.")
        fig_height = MAX_HEIGHT_INCHES

    params = {'backend': 'ps',
              'text.latex.preamble': ['\\usepackage{gensymb}'],
              'axes.labelsize': 8, # fontsize for x and y labels (was 10)
              'axes.titlesize': 8,
              'font.size': 8, # was 10
              'legend.fontsize': 8, # was 10
              'xtick.labelsize': 8,
              'ytick.labelsize': 8,
              'text.usetex': True,
              'figure.figsize': [fig_width,fig_height],
              'font.family': 'serif'
    }

    matplotlib.rcParams.update(params)

def format_axes(ax):
    for spine in ['top', 'right']:
        ax.spines[spine].set_visible(False)

    for spine in ['left', 'bottom']:
        ax.spines[spine].set_color(SPINE_COLOR)
        ax.spines[spine].set_linewidth(0.5)

    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    for axis in [ax.xaxis, ax.yaxis]:
        axis.set_tick_params(direction='out', color=SPINE_COLOR)

    return ax



def plot_task(task):
    profiles = task.get_profiles()
    for profile in profiles:
        plot_profile(profile)
        input("next?")

def plot_profile(profile, fname='default'):
    print(profile)
    high_energy, low_time = profile['min_time']
    #plt.plot([low_time], [high_energy], 'ro') # red: fast but costly
    low_energy, high_time = profile['min_energy']
    #plt.plot([high_time], [low_energy], 'bo') # blue: slow but energy efficient
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

def plot_alternate(result_file, profile, altspec):
    max_energy = profile['max_energy']
    min_energy = profile['min_energy'][0]
    max_time = profile['max_time']
    min_time = profile['min_time'][0]
    plt.xlabel('Trial (\#)')
    plt.ylabel('Time (s)')
    if altspec == 'energy':
        plt.ylabel('Energy (J)')
    with open(result_file, 'r') as results:
        times = []
        energies = []
        sizes = []
        colors = []
        alphas = []
        for result_line in results:
            if result_line[0] and result_line[0].isdigit() or result_line[0] == '-':
                parts = [float(x) for x in result_line.split()]
                color = 'black'
                alpha = 1.0
                if parts[0] <= 0:
                    color = 'orange'
                # format is STEPS, ENERGY, CLOCK, MIN_ENERGY
                energy_size = 1 + ((((parts[1] - min_energy) ** 2) / ((max_energy - min_energy) ** 2)) * 100)
                time_size = 1 + (((parts[2] ** 2) / (max_time ** 2)) * 25)
                if altspec == 'energy':
                    energies.append(parts[1])
                    sizes.append(time_size)
                else:
                    times.append(parts[2])
                    sizes.append(energy_size)
                colors.append(color)
    if altspec == 'energy':
        plt.scatter(range(1, 1+len(energies)), energies, marker='x', edgecolor=colors, facecolor=colors, s=sizes, alpha=0.9)
    else:
        plt.scatter(range(1, 1+len(times)), times, marker='.', edgecolor=colors, facecolor=colors, s=sizes, alpha=0.9)
    plt.xlim(xmin=0, xmax=max(len(times), len(energies))+2)
    plt.ylim(ymin=0, ymax=profile['max_time']*1.05)
    if altspec == 'energy':
        plt.ylim(ymin=0, ymax=profile['max_energy']*1.05)
    import os
    imgfile, _ = os.path.splitext(result_file)
    plt.tight_layout()
    format_axes(plt.gca())
    plt.savefig(imgfile + '_alt' + altspec + '.png', bbox_inches='tight', dpi=1000)

def plot_results_on_profile(result_file, profile):
    
    latexify()
    
    # very green = early result
    # very dark green = late result
    # more red = more recent result
    def next_color(current):
        green_str = current[3:5]
        green = int(green_str, 16) - 3
        red_str = current[1:3]
        red = int(red_str, 16)
        blue = int(current[5:7], 16)
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
                plt.plot([parts[2]], [parts[1]], '.', color=current_color)
                xmin, xmax = plt.xlim()
                ymin, ymax = plt.ylim()
                plt.xlim(xmax=max(xmax, parts[2]*1.05))
                plt.ylim(ymax=max(ymax, parts[1]*1.05))
    # include the cutoff in the plot
    plt.xlim(xmax=max(xmax, profile['max_time']))
    plt.ylim(ymax=max(ymax, profile['power'] * profile['max_time']))
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
    #plt.xlim(xmin=0, xmax=50); plt.ylim(ymin=0, ymax=750) # domain_20
    #plt.xlim(xmin=0, xmax=25); plt.ylim(ymin=0, ymax=225) # domain_10
    import os
    imgfile, _ = os.path.splitext(result_file)
    plt.tight_layout()
    format_axes(plt.gca())
    plt.savefig(imgfile + '.png', bbox_inches='tight', dpi=1000)

def main(args):
    if args.filename:
#       task = samples.sample_system_1D_plotter(max_power=200, default_start=20, default_delta=0)[0] # easy
#       task = samples.sample_system_1Db_plotter2(max_power=200, default_start=20, default_delta=0)[0] # hard
        task, _ = samples.sample_N_task(50, delta=0) # N-dimensions task
        for var in task.all_objects():
            if hasattr(var, 'name'):
                print("{}: {}".format(var.name, var))
        profile = task.get_profile()
        profile['curve'] = task.calc_curve_time(0.01, profile['min_energy'][1])
        if not args.alternate:
            plot_results_on_profile(args.filename, profile)
        else:
            latexify()
            plot_alternate(args.filename, profile, args.alternate)


if __name__ == '__main__':
    import samples
    parser =  argparse.ArgumentParser(description="plot the power profile") 
    parser.add_argument("filename", metavar="FILE")
    parser.add_argument("--alternate", "-a", nargs='?', help="plot energy as size")
    args = parser.parse_args()
    try:
        main(args)
#       plt.show()
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

