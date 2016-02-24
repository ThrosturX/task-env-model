import math
import numpy
import random
import traceback

def warning(msg):
    try:
        DEBUG
    except NameError:
        return
    else:
        if DEBUG:
            print(msg)

class AbstractTaskEnvironmentVariable(object):
    """A class to encapsulate variables in the environment"""
    def __init__(self, start_value, start_delta, goal_value, goal_delta):
        self.value = start_value + random.uniform(-start_delta, start_delta)
        self.goal = goal_value
        self.goal_delta = goal_delta

    def in_goal(self):
        """Utility to check if the variable is in a goal state"""
        lower = self.goal - self.goal_delta
        upper = self.goal + self.goal_delta
        return self.in_range(lower, upper)

    def in_range(self, lower, upper):
        """Utility to check if a variable is in a range"""
        return self.goal >= lower and self.goal <= upper

    def dist_to_goal(self):
        dist1 = abs(self.goal + self.goal_delta - self.value)
        dist2 = abs(self.goal - self.goal_delta - self.value)
        return min(dist1, dist2)

    def natural_transition(self, delta_time):
        pass # no return value needed #(?)
#       return self.value # no change

class UnboundedTaskEnvironmentVariable(AbstractTaskEnvironmentVariable):
    """'Physical' variables, 'slider'
    This class encapsulates the basic physics behind the variables
    """
    def __init__(self, start_value, start_delta, goal_value, goal_delta, mass=1, initial_velocity=0):
        super().__init__(float(start_value), start_delta, goal_value, goal_delta)
        self.velocity = initial_velocity
        self.mass = mass
        self.friction_static = 0
        self.friction_kinetic = 0
        self.angle = 0 # radians 
        self.gravity = 0

    def natural_transition(self, delta_time=1):
        if self.velocity == 0 and self.angle == 0:
            return self.value # not in any state of change
        # before updating, we must decelerate due to friction
        acceleration, clamp = self.calc_accel_clamp()
        # update velocity
        self.velocity = clamp(0, self.velocity + float(acceleration * delta_time))
        # update value
        self.value = self.value + float(self.velocity * delta_time) # value updated based on current velocity
        return self.value

    def min_energy(self, delta_time=1):
        """Determine how much energy must be supplied to have any effect with a time resolution of delta_time"""
        acceleration = self.calc_acceleration()
        min_velocity = acceleration * delta_time
        E_kin = (min_velocity ** 2) * self.mass / 2
        return E_kin

    def min_time_resolution(self, max_energy):
        """Determine the mininmum time resolution required if we can only deliver max_energy joules per second"""
        acceleration = self.calc_acceleration()
        max_velocity = math.sqrt(2 * max_energy / self.mass)
        delta_time = max_velocity / acceleration
        return delta_time

    def calc_accel_clamp(self):
        """Calculate the acceleration due to gravity and friction"""
        F_grav = self.mass * self.gravity
        F_norm = F_grav * math.cos(self.angle)
        F_friction = self.friction_static * F_norm
        F_grav_accel = F_grav * math.sin(self.angle)
        acceleration = -F_grav_accel # (because we assume 'upwards' is 'increasing')
        if self.velocity > 0:
            clamp = max
            acceleration += -(F_friction / self.mass) # acceleration should be negative, but might be positive
        else:
            clamp = min
            acceleration += (F_friction / self.mass) # acceleration should be negative, but might be positive
        return acceleration, clamp

    def calc_acceleration(self):
        accel, _ = self.calc_accel_clamp()
        return accel

    def apply_force(self, newtons, gravity, delta_time=1):
        """Apply a force of N newtons on the variable in a single step"""
        # No force applied, then nothing happens
        if newtons == 0:
            return self.value
        # calculate the frictional force
        F_grav = self.mass * gravity
        F_norm = F_grav * math.cos(self.angle)
        mu = self.friction_static
        if self.velocity != 0:
            mu = self.friction_kinetic
        F_friction = mu * F_norm
        # calculate the net force
        if newtons < 0: # negative force, i.e. force in opposite direction
            F_friction *= -1 # friction is negative if force is negative (using this convention)
        F_net = newtons - F_friction
        assert(F_net < newtons), "Net force is greater than input force!\n({}, {})".format(F_net, newtons)
        # calculate the new velocity based on the force
        # F = m / a => a = m / F
        acceleration = self.mass / F_net 
        self.velocity = self.velocity + acceleration * delta_time
        # move the slider based on its velocity
        self.value = self.value + self.velocity * delta_time
        return self.value

    # If there is no kinetic friction, the energy required is at the limit of zero and time depends on how much energy you put in
    def calc_drift(self):
        """Calculate the kinetic energy, initial velocity and time required for the item to drift into the goal"""
        distance = abs(self.goal - self.value)#- self.goal_delta
                 # F_Grav * angle * mu ## we assume no static friction for optimization
        F_fric   = (self.mass * self.gravity) * math.cos(self.angle) * self.friction_kinetic
        accelneg = -F_fric / self.mass
        # calculate the energy required to accelerate the item to a velocity that will drift into the final position
        drift_vel = math.sqrt(-2 * accelneg * distance)
        drift_kin = 0.5 * self.mass * (drift_vel ** 2)
        drift_time = -drift_vel / accelneg
        return (drift_kin, drift_vel, drift_time)

    def calc_min_energy(self, max_time):
        distance = abs(self.goal - self.value)
    ### F_Fric   = F_Grav * angle * mu ## we assume no static friction for optimization
        F_fric   = (self.mass * self.gravity) * math.cos(self.angle) * self.friction_kinetic
        accelneg = -F_fric / self.mass
        # calculate the energy required to accelerate the item to a velocity that will drift into the final position
        drift_kin, drift_vel, drift_time = self.calc_drift()
        # if it takes too long, increase the energy required appropriately
        if drift_time > max_time and max_time > 0:
            min_vel = distance / max_time - accelneg * max_time / 2
            E_push = self.mass * (min_vel ** 2) / 2
            # now we need to calculate the cost of stopping it
            vel_time_t = min_vel + accelneg * max_time
            E_stop = abs(self.mass * (vel_time_t ** 2) / 2)

            return E_push + E_stop, (E_push, E_stop)
        return drift_kin, (drift_kin, 0)

    def calc_min_time(self, max_joules):
        distance = abs(self.goal - self.value)
        drift_kin, drift_vel, drift_time = self.calc_drift()
        assert(max_joules > drift_kin), "At least {} joules are needed to accomplish the task!".format(drift_kin)
        # figure out how much faster we can make it
        F_fric   = (self.mass * self.gravity) * math.cos(self.angle) * self.friction_kinetic
        accelneg = -F_fric / self.mass
        E_fric = F_fric * distance

        E_push = (max_joules + abs(E_fric)) / 2
        velocity = math.sqrt(2 * E_push / self.mass)
        time = (-velocity + math.sqrt((velocity ** 2) + 2 * distance * accelneg)) / accelneg

        return time

class TaskEnvironmentVariable(UnboundedTaskEnvironmentVariable):
    def set_bounds(self, lower, upper):
        self.lower_bound = lower
        self.upper_bound = upper

    def natural_transition(self, delta_time=1):
        value = super().natural_transition(delta_time)
        old_value = self.value
        self.value = max(self.lower_bound, min(self.upper_bound, value))
        if old_value != self.value:
            self.velocity = 0
        return value

class TaskEnvironmentTransition(object):
    """Transition function allowing mutations to variables"""
    def __init__(self, affected_variables, transition_function):
        self.affected_variables = affected_variables
        self.transition = transition_function

    def apply_transition(self, *args):
        """Apply a transition function to variables and return the resulting variable set"""
        if self.affected_variables: # affected_variables get unpacked
            new_variables = self.transition(*self.affected_variables, *args)
        else:
            assert(args), "This transition requires arguments\n({})".format(self.transition)
            new_variables = self.transition(*args)
        return new_variables

# TODO: this is a system, and needs to be broken down into System and TEM
class TaskEnvironmentModel(object):
    """Represent task environment models as E = {V,T}"""
    def __init__(self, variables, transitions):
        self.variables = variables
        self.transitions = transitions
        self.clock = 0

    def tick(self, delta_time):
        """Affect every variable with the natural change caused by delta_time seconds elapsing"""
        for variable in self.variables:
            variable.natural_transition(float(delta_time))
        self.clock += delta_time

    def get_transitions(self):
        return self.transitions

    def fire_transition(self, transition, *args):
        assert transition in self.transitions, "The transition must be a part of the task environment model to be fired"
        transition.apply_transition(*args)

    def energy_needed(self):
        """Calculate the necessary power to move every variable into a goal position"""
        joules = 0
        for var in self.variables:
            if var.friction_kinetic > 0:  # kinetic friction present, calculate drift energy
                E_min, _, _ = var.calc_drift()
                joules += E_min
            elif var.calc_acceleration() != 0:
                # the variable is (probably) moving away from its goal constantly,
                # so calculate the energy to move it directly to the goal
                a = var.calc_acceleration()
                d = var.dist_to_goal()
                # edge case #1: acceleration moves the value into the goal at some point in the future
                if a > 0 and var.value < var.goal_value \
                or a < 0 and var.value < var.goal_value:
                    joules == 0
                else:
                    # work = force x distance
                    watts = var.mass * a * d
                    joules += watts # cheating a little bit here not sure if this is OK # TODO
            else: # There is no kinetic friction, so essentially this variable is 'free' apart from the initial push and equal stop, which we won't count since it depends on a time resolution
                joules += 0 # We might want to add some "minimum power"
        return joules

    def minimum_time(self, motors):
        """Calculate which variable takes the longest to move into the goal position given a set of motors"""
        def calc_time(var, motor):
            if isinstance(motor, MultiMotor):
                power = motor.targetmap.get(var) * motor.max_power
            else:
                power = motor.max_power
            # check if it needs to be reversed
            if var.value > var.goal:
                if not motor.reversible:
                    return
                power = power * motor.reverse_power_ratio
            # calculate the time needed with this power
            dist_left = var.dist_to_goal()
            time = (((3 * dist_left) ** 2) * var.mass ) ** (1/3) / 2 * (power ** (1/3))
            def rt23(x):
                return x ** (2/3)
            time = (rt23(3) * (var.mass ** (1/3)) * rt23(dist_left)) / (2 * (power ** (1/3)))
            return time
        varmap = {}
        for motor in motors:
            transition = motor.activation
            affected = transition.affected_variables
            for var in affected:
                varmap[var] = calc_time(var, motor)
        for k,d in varmap.items():
            print("{}: {}".format(k,d))
        if varmap:
            longest_time = max(varmap.values())
        # TODO: Something is wrong here or elsewhere, look at paper
        # https://kb.osu.edu/dspace/bitstream/handle/1811/2458/V30N04_218.pdf

    def solved(self):
        for variable in self.variables:
            lower_bound = variable.goal - variable.goal_delta
            upper_bound = variable.goal + variable.goal_delta
            if variable.value < lower_bound or variable.value > upper_bound:
                return False
        return True   

class Motor(object): # 'Actuator'
    """A class to encapsulate TaskEnvironmentTransitions that affect TaskEnvironmentVariables"""
    def __init__(self, target, properties):
        assert(hasattr(target, "velocity")), "{} doesn't have a velocity.".format(target)
        assert(hasattr(target, "mass")), "{} doesn't have a mass.".format(target)
        self.activation = TaskEnvironmentTransition([target], self.inject_energy)
        self.init_properties(properties)
        self.kinetic_energy = 0

    def init_properties(self, properties={}):
        """Motors have some default properties that can be overwritten"""
        self.max_power = properties.get('max_power', 100)
        self.reversible = properties.get('reversible', True)
        self.reverse_power_ratio = properties.get('reverse_power_ratio', 1.0)

    # Note: watts express the rate of energy transfer with respect to time
    def activate(self, watts, delta_time=1):
        joules = min(self.max_power, max(-self.max_power * self.reverse_power_ratio, watts)) * delta_time # don't allow overpower
        # conditional checks regarding whether it can be activated in this way
        if watts < 0 and not self.reversible:
            warning("Attempting to reverse a unidirectional motor")
        if joules != watts * delta_time:
            warning("Attempting to activate a {} W motor with {} W".format(self.max_power, watts))
        if watts < 0 and watts > self.reverse_power_ratio * self.max_power:
            warning("Reversing a {} W motor at {} W with a ratio of {}".format(self.max_power, watts, self.reverse_power_ratio))
        self.activation.apply_transition(joules)

    def inject_energy(self, target, joules): # joules can be positive or negative
        """Increase the kinetic energy directly by some amount of joules (sign indicates direction)
        If the energy isn't sufficient to move the target, nothing happens (it is wasted).
        """
        E_sign = 1 if (target.velocity > 0) else -1
        E_kin = E_sign * 0.5 * target.mass * (target.velocity ** 2)
        E_new = E_kin + joules + self.kinetic_energy
        # rest is true if the velocity is ever zero
        rest = (target.velocity == 0) or abs(E_kin - E_new) > max(abs(E_kin), abs(E_new))
        # Minimum amount of energy to *increase* its velocity while *at rest*
        E_min = target.friction_static * target.mass * target.gravity * math.cos(target.angle)
        
        if abs(E_new) < abs(E_min) and rest:
            # store it as kinetic energy in the motor that might get used later
            self.kinetic_energy = E_new
            print("{} stored since it is less than {}".format(E_new, E_min))
            return # no effect
        if E_new < 0: # negative velocity
            target.velocity = -math.sqrt(2 * -E_new / target.mass) # velocity has been updated
        else: # default
            target.velocity = math.sqrt(2 * E_new / target.mass) # velocity has been updated

class MultiMotor(Motor):
    """A class that extends Motors to allow modifications to multiple TaskEnvironmentVariables"""
    def __init__(self, targetmap, properties):
        targets = []
        for target, weight in targetmap.items():
            assert(hasattr(target, "velocity")), "{} doesn't have a velocity.".format(target)
            assert(hasattr(target, "mass")), "{} doesn't have a mass.".format(target)
            targets.append(target)
        self.activation = TaskEnvironmentTransition([targets], self.distribute_energy)
        self.init_properties(properties)
        self.efficiency = self._calc_efficiency(targetmap) # tells you how much power goes out compared to in
        self.targetmap = targetmap

    def activate(self, watts, delta_time=1):
        joules = min(self.max_power, max(-self.max_power * self.reverse_power_ratio, watts)) * delta_time # don't allow overpower
        if watts < 0 and not self.reversible:
            warning("Attempting to reverse a unidirectional motor")
        if joules != watts * delta_time:
            warning("Attempting to activate a {} W motor with {} W".format(self.max_power, watts))
        if watts < 0 and watts > self.reverse_power_ratio * self.max_power:
            warning("Reversing a {} W motor at {} W with a ratio of {}".format(self.max_power, watts, self.reverse_power_ratio))
        self.activation.apply_transition(self.targetmap, joules)

    def distribute_energy(self, targetmap, joules):
        """Distributes the energy between the targets based on their weighting"""
        for target, weight in targetmap.items():
            self.inject_energy(target, joules * weight)

    def _calc_efficiency(self, targetmap):
        eff = 0
        for value in targetmap.values():
            eff += value
        return eff

class Sensor(object):
    """A class that reads variables and rounds them after optionally distorting them
    Distortion is the maximum amount that the sensor reading can deviate from the correct value
    """
    def __init__(self, observed_variable, rounding_digits=0, distortion=0):
        self.variable = observed_variable
        self.n_digits = rounding_digits
        self.distortion = distortion

    def read(self):
        """Return a perceived value from the sensor"""
        if type(self.variable) == Sensor:
            value = self.variable.read()
        else: 
            value = self.variable.value
        value = value + random.uniform(-self.distortion, self.distortion)
        return round(value, self.n_digits)

def simple_whack_mole():
    X = TaskEnvironmentVariable(15, 5, 300, 10)
    X.set_bounds(0, 500)
    X.gravity = 10
    X.friction_kinetic = 0.2
    X.friction_static = 0.3
    Y = TaskEnvironmentVariable(25, 5, 200, 10)
    Y.set_bounds(0, 500)
    Y.gravity = 10
    Y.friction_kinetic = 0.2
    Y.friction_static = 0.3
    V = [X, Y]
    V = [X]
    motor_properties = {'max_power': 100}

    mx = Motor(X, motor_properties)
    my = Motor(Y, motor_properties)
    T = [mx.activation, my.activation]
    
    env = TaskEnvironmentModel(V, T)

#   min_energy = X.calc_min_energy(60)
#   print("{} solvable in {} seconds.".format("X", min_time))
#   print("{} solvable with {} Joules.".format("X", min_energy))
    drift_kin, drift_vel, drift_time = X.calc_drift()
    max_time = 10
    min_energy, min_e_factors = X.calc_min_energy(10)
    min_time = X.calc_min_time(min_energy)
    E_start = min_e_factors[0]
    E_stop  = min_e_factors[1]
    print("Need {:.2f} J to move it in {} seconds".format(min_energy, max_time))
    print("The distance is {}".format(X.goal - X.value))
    print("{:.2f} m/s v0 should move it in {:.2f} s ({:.2f} J)".format(drift_vel, drift_time, drift_kin))
    print("t  0 vel: {}, val: {}".format(X.velocity, X.value))
    for i in range(int(drift_time)):
        print("t {:2d} vel: {:.2f}, val: {:.2f}".format(i, X.velocity, X.value))
        if i == 0:
            mx.activate(E_start)
            print("Activated {} W".format(E_start))
        elif i == max_time:
            mx.activate(-E_stop)
            print("Activated {} W".format(-E_stop))
        env.tick(1)
    print("t {:2d} vel: {:.2f}, val: {:.2f}".format(i, X.velocity, X.value))
    # agent code
    import simple_agent
    sx = Sensor(X, -1, 3)
    sy = Sensor(Y, -1, 3)
    sensors = [sx, sy]
    motors = [mx, my]
    agent = simple_agent.CheatingTaskEnvironmentAgent(env, sensors, motors)
    # let the agent try to solve the problem
    no_steps = 0
    print("Distance left: {}".format(X.dist_to_goal()))
    return
    while not env.solved():
        agent.perform(1)
        no_steps += 1
        for s_no, sensor in enumerate(sensors):
            print("| S{}: {:.2f} / {} ".format(s_no, sensor.read(), sensor.variable.goal), end='')
        print(" {} \r".format(no_steps), end='')
    else:
        print("\nProblem solved in {} steps".format(no_steps))

if __name__ == '__main__':
    # no arguments yet
    try:
        simple_whack_mole()
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


