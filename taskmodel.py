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

class AbstractVariable(object):
    """Implements syntactic sugar allowing easy variable manipulation"""
    def __add__(self, other):
        return self.value + other

    def __radd__(self, other):
        return self.value + other

    def __sub__(self, other):
        return self.value - other

    def __rsub__(self, other):
        return - self.value + other

    def __lt__(self, other):
        return self.value < other

    def __gt__(self, other):
        return self.value > other

    def __str__(self):
        return "[{}]".format(self.value)

    def get_name(self):
        try:
            return self.name
        except AttributeError:
            return repr(self)

class UnboundedTaskEnvironmentVariable(AbstractVariable):
    """'Physical' variables, 'slider'
    This class encapsulates the basic physics behind the variables
    """
    def __init__(self, start_value, start_delta, mass=1, initial_velocity=0):
        self.value = start_value + random.uniform(-start_delta, start_delta)
        self.velocity = initial_velocity
        self.mass = mass
        self.friction_static = 0
        self.friction_kinetic = 0
        self.angle = 0 # radians 
        self.gravity = 0
        self.affectors = []
        self.locked = False

    def natural_transition(self, delta_time=1):
        if self.locked:
            warning("{} is locked".format(self.name))
            return self.value # no change if locked
        if self.velocity == 0 and self.angle == 0 and not self.affectors:
            return self.value # not in any state of change
        # update velocity
        old_vel = self.velocity
        self.velocity = self.calc_velocity(delta_time)
        # update value ( s1 = s0 + v dt )
        self.value = self.value + float(self.velocity * delta_time) # value updated based on current velocity
        return self.value

    def calc_velocity(self, delta_time=1):
        # Calculates v(t+dt), given that v(t) = self.velocity
        power = 0
        for affector in self.affectors:
            power += affector.component(self)
        final_sign = 1
        vel_rel = self.velocity
        sign_p = 1 if power >= 0 else -1
        sign_v = 1 if self.velocity >= 0 else -1
        F_comp_grav = - self.mass * self.gravity * math.sin(self.angle)
        F_fric = self.mass * self.gravity * math.cos(self.angle) * self.friction_kinetic
        F_net = F_comp_grav - F_fric 

        # Branching!!
        if self.velocity == 0:
            # Limited approximation method.
            # Calculate F_static in newtons and check if F_power is higher, if not, nothing happens (velocity stays at zero)
            # Formulas:
            # F_power = a_power * m
            # a_power = sqrt(P/2mt)
            # F_static = mg cos(angle) * mu_static
            ###
            F_static = self.mass * self.gravity * math.cos(self.angle) * self.friction_static
            F_net = F_net * sign_v # is this necessary??
            a_power = math.sqrt(abs(power)/(2 * self.mass * delta_time))
            F_power = a_power * self.mass
            # IF F_power > F_static THEN F_tot = F_power - F_net (F_net is the force due to friction and gravity)
            if abs(F_power) > abs(F_static):
                # NOW a = F_tot / m and v = a * dt
                    F_tot = F_power - F_net
                    acceleration = (F_tot / self.mass) * sign_p
                    velocity = acceleration * delta_time
            # there might still be some acceleration due to gravity...
            elif abs(F_comp_grav) > abs(F_static):
                # TODO THIS PART
                F_tot = F_comp_grav + F_power - F_fric
                acceleration = (F_tot / self.mass) * sign_p
                velocity = acceleration * delta_time
            else:
                velocity = 0
        else:
            # Friction acts in the opposite direction of velocity
            opps = -1 if power < 0 and self.velocity < 0 else 1
            F_net = F_net * sign_v * opps
            # m dv / dt = P/v+F => dv = (P/v+F)dt /m
            dv = ((power / vel_rel) + F_net) * delta_time / self.mass
#           input("v: {:.2f}, dv: {:.2f}".format(self.velocity, dv))
            velocity = self.velocity + dv * opps
            # If there exists a point 0 < c < dt where v(c) == 0, then we need to go back to
            # the case where velocity is zero.                      NOTE NOTE NOTE NOTE NOTE
            # We can either stop the item for this time slice,      NOTE NOTE NOTE NOTE NOTE
            # or split the calculation up further                   NOTE NOTE NOTE NOTE NOTE
            # by calculating the exact time it stops and continuing from there,    NOTE NOTE
            # but I leave this as future work
            if velocity > 0 and sign_v < 0: # if the sign changed, then v(c) == 0 exists for 0 < c < dt
                velocity = +0.0

        return velocity

    def lock(self):
        self.locked = True

    def unlock(self):
        self.locked = False

    # TODO: marked for refactoring
    def min_energy(self, delta_time=1):
        """Determine how much energy must be supplied to have any effect with a time resolution of delta_time"""
        acceleration = self.calc_acceleration()
        min_velocity = acceleration * delta_time
        E_kin = (min_velocity ** 2) * self.mass / 2
        return E_kin

    # TODO: marked for refactoring
    def min_time_resolution(self, max_energy):
        """Determine the mininmum time resolution required if we can only deliver max_energy joules per second"""
        acceleration = self.calc_acceleration()
        max_velocity = math.sqrt(2 * max_energy / self.mass)
        delta_time = max_velocity / acceleration
        return delta_time

    # TODO: NEEDS TESTING
    def calc_acceleration(self):
        """Calculate the acceleration due to gravity and friction"""
        F_grav = self.mass * self.gravity
        F_norm = F_grav * math.cos(self.angle)
        F_fric = self.friction_kinetic * F_norm
        F_comp_grav = - F_grav * math.sin(self.angle)
        F_tot = F_comp_grav
        F_sign = 1 if F_tot >= 0 else -1
        F_net = (abs(F_tot) - abs(F_fric)) * F_sign
        acceleration = F_net / self.mass
        # TODO: NEEDS TESTING
        if abs(F_tot) < abs(F_fric): # force can't overcome friction
            if self.velocity == 0:
                acceleration = 0
            elif self.velocity > 0:
                acceleration = -F_fric + F_tot / self.mass
            elif self.velocity < 0:
                acceleration = +F_fric + F_tot / self.mass
        return acceleration

    # TODO: marked for inspection
    def calc_power_limits(self, delta_time=1):
        F_grav = self.mass * self.gravity
        F_norm = F_grav * math.cos(self.angle)
        F_fric_kinetic = self.friction_kinetic * F_norm
        F_fric_static  = self.friction_static  * F_norm
        kinetic_accel = F_fric_kinetic / self.mass
        static_accel  = F_fric_static  / self.mass
        grav_accel    = F_grav * math.sin(self.angle) / self.mass
        P_kine = 2 * self.mass * delta_time * (kinetic_accel ** 2)
        P_stat = 2 * self.mass * delta_time * (static_accel  ** 2)
        P_grav = 2 * self.mass * delta_time * (grav_accel ** 2)
        return P_stat, P_kine, P_grav

    # TODO: needs testing
    # If there is no kinetic friction, the energy required is at the limit of zero and time depends on how much energy you put in
    def calc_drift(self, goal, delta=0):
        """Calculate the kinetic energy, initial velocity and time required for the item to drift into the goal"""
        distance = abs(goal - self.value) - delta
                 # F_Grav * angle * mu ## we assume no static friction for optimization
        F_fric   = (self.mass * self.gravity) * math.cos(self.angle) * self.friction_kinetic
        accelneg = -F_fric / self.mass
        # calculate the energy required to accelerate the item to a velocity that will drift into the final position
        work = F_fric * distance
        drift_kin = abs(work) # 0.5 * self.mass * (drift_vel ** 2)
        # TODO: vel and time need testing
        drift_vel = math.sqrt((2 * drift_kin)/self.mass)
        drift_time = -drift_vel / accelneg
        return (drift_kin, drift_vel, drift_time)

    def calc_min_energy(self, max_time):
        """Calculate the energy required to accelerate the variable to a velocity that will drift into the final position"""
        drift_kin, drift_vel, drift_time = self.calc_drift()
        # if it takes too long, increase the energy required appropriately
        if drift_time > max_time and max_time > 0:
            distance = abs(self.goal - self.value)
            F_fric   = (self.mass * self.gravity) * math.cos(self.angle) * self.friction_kinetic
            accelneg = -F_fric / self.mass
            min_vel = distance / max_time - accelneg * max_time / 2
            E_push = self.mass * (min_vel ** 2) / 2
            # now we need to calculate the cost of stopping it
            vel_time_t = min_vel + accelneg * max_time
            E_stop = abs(self.mass * (vel_time_t ** 2) / 2)

            return E_push + E_stop, (E_push, E_stop)
        return drift_kin, (drift_kin, 0)

    def calc_min_time(self, max_joules):
        """Calculate the number of seconds it will take to move the variable to the final position given a limited amount of energy""" 
        distance = abs(self.goal - self.value)
        drift_kin, drift_vel, drift_time = self.calc_drift()
        assert(max_joules > drift_kin), "At least {} joules are needed to accomplish this subtask!".format(drift_kin)
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
            self.velocity = 0 # if we reach the bounds, we stop moving (something else would need to detect actual collisions and deal with them)
        return value

class TaskEnvironmentTransition(object):
    """Transition function allowing mutations to variables"""
    def __init__(self, affected_variables, transition_function):
        self.affected_variables = affected_variables
        self.transition = transition_function
        def precondition(): return True
        self.precondition = precondition

    def set_precondition(self, precondition):
        self.precondition = precondition

    def apply_transition(self, *args):
        """Apply a transition function to variables and return the resulting variable set"""
        if not self.precondition():
            return None
        if self.affected_variables: # affected_variables get unpacked
            new_variables = self.transition(*self.affected_variables, *args)
        else:
            assert(args), "This transition requires arguments\n({})".format(self.transition)
            new_variables = self.transition(*args)
        return new_variables

class TaskEnvironmentGoal(object):
    def __init__(self, target, goal_value, goal_delta):
        # maybe check the types here
        self.target = target
        self.value = goal_value
        self.delta = goal_delta
        self.satisfied = False
        self.prerequisite = None

    def set_prerequisite(self, goal):
        self.prerequisite = goal

    def assess(self):
        if self.current_condition():
            if not self.prerequisite                   \
            or (self.prerequisite                      \
            and self.prerequisite.current_condition()  \
               ):
                self.satisfied = True

    def current_condition(self):
        if self.target < self.value + self.delta \
        and self.target > self.value - self.delta:
            return True

# SENCTOR
class TaskEnvironmentSystem(object):
    """Class to encapsulate the behavior of a variable. Simplifies interaction.
    Systems can have variables, transitions, motors and sensors.
    """
    def __init__(self, variables, transitions=[], motors=[], sensors=[], systems=[]):
        if not hasattr(variables, '__iter__'):
            variables = [variables]
        if not hasattr(transitions, '__iter__'):
            transitions = [transitions]
        if not hasattr(motors, '__iter__'):
            motors = [motors]
        if not hasattr(sensors, '__iter__'):
            sensors = [sensors]
        if not hasattr(systems, '__iter__'):
            systems = [systems]
        self.variables = variables
        self.transitions = transitions
        self.motors = motors
        self.sensors = sensors
        self.systems = systems

    def satisfies(self, solution):
        result = solution(*self.variables)
        return result

    def all_variables(self):
        if not self.systems:
            return self.variables
        all_variables = set(self.variables.copy())
        for system in self.systems:
            all_variables = all_variables.union(system.all_variables())
        return list(all_variables)

    def all_motors(self):
        if not self.systems:
            return self.motors
        all_motors = set(self.motors.copy())
        for system in self.systems:
            all_motors = all_motors.union(system.all_motors())
        return list(all_motors)

    def all_sensors(self):
        if not self.systems:
            return self.sensors
        all_sensors = set(self.sensors.copy())
        for system in self.systems:
            all_sensors = all_sensors.union(system.all_sensors())
        return list(all_sensors)

    def all_systems(self):
        if not self.systems:
            return [self]
        return list(self.systems)

    def lock_variables(self):
        for variable in self.variables:
            variable.lock()

    def unlock_variables(self):
        for variable in self.variables:
            variable.unlock()

class TaskEnvironmentModel(object):
    """Represent task environment models as E = {V,T}
    With some abstractions.
    """
    def __init__(self, environment, solution):
        self.environment = environment  # should be a system
        self.solution = solution        # should be a list of goals
        self.clock = 0
        self.dt = 0.001
        self.solution_score = 0.0 # the model starts as 'unsolved' with respect to any solutions

    def all_variables(self):
        all_vars = set()
        for variable in self.environment.all_variables():
            all_vars.add(variable)
        return all_vars

    def tick(self, delta_time):
        """Affect every variable with the natural change caused by delta_time seconds elapsing"""
        time_passed = 0
        while time_passed < delta_time:
            time_passed += self.dt
            for variable in self.all_variables():
                variable.natural_transition(float(self.dt))
                self.check_for_solutions()
        self.clock += delta_time 

    # a solution is set of booleans representing achieved goals
    # for now solutions are simple booleans
    def check_for_solutions(self):
        if self.solved():
            self.solution_score = 1.0

    def motors(self):
        return self.environment.all_motors()

    def solved(self):
        for goal in self.solution:
            goal.assess()
            if not goal.satisfied:
                return False
        return True

    def goal_vars(self):
        g_vars = []
        for goal in self.solution:
            element = goal.target, goal.value, goal.delta
            g_vars.append(element)
        return g_vars

    def energy_needed(self):
        """Calculate the necessary power to move every variable into a goal position"""
        joules = 0
        for var, goal, delta in self.goal_vars():
            if var.friction_kinetic > 0:  # kinetic friction present, calculate drift energy
                E_min, _, _ = var.calc_drift(goal, delta)
                joules += E_min
            elif var.calc_acceleration() != 0:
                # the variable is (probably) moving away from its goal constantly,
                # so calculate the energy to move it directly to the goal
                a = var.calc_acceleration()
                d = abs(var - goal)
                # edge case #1: acceleration moves the value into the goal at some point in the future
                if a > 0 and var < goal \
                or a < 0 and var < goal:
                    joules == 0
                else:
                    # work = force x distance
                    work = abs(var.mass * a * d)
                    joules += work 
            else: # There is no kinetic friction, so essentially this variable is 'free' apart from the initial push and equal stop, which we won't count since it depends on a time resolution
                joules += 0 # We might want to add some "minimum power"
        return joules

class Motor(object): # 'Actuator'
    """A class to affect TaskEnvironmentVariables"""
    def __init__(self, target, properties):
        assert(hasattr(target, "velocity")), "{} doesn't have a velocity.".format(target)
        assert(hasattr(target, "mass")), "{} doesn't have a mass.".format(target)
        self.target = target
        self.init_properties(properties)
        self.power_level = 0
        target.affectors.append(self)

    def init_properties(self, properties={}):
        """Motors have some default properties that can be overwritten"""
        self.max_power = properties.get('max_power', 100)
        self.reversible = properties.get('reversible', True)
        self.reverse_power_ratio = properties.get('reverse_power_ratio', 1.0)

    # Note: watts express the rate of energy transfer with respect to time
    def activate(self, watts):
        # conditional checks regarding whether it can be activated in this way
        if watts < 0 and not self.reversible:
            warning("Attempting to reverse a unidirectional motor")
        if watts > self.max_power:
            warning("Attempting to activate a {} W motor with {} W".format(self.max_power, watts))
        if watts < 0 and watts > self.reverse_power_ratio * self.max_power:
            warning("Reversing a {} W motor at {} W with a ratio of {} (max reversal power is {})".format(self.max_power, watts, self.reverse_power_ratio, self.reverse_power_ratio * self.max_power))
        self.power_level = watts

    def component(self, item):
        if item == self.target:
            return self.power_level
        warning("Attempting to find the component of a motor that doesn't target {}".format(item))
        return 0

class MultiMotor(Motor):
    """A class that extends Motors to allow modifications to multiple TaskEnvironmentVariables"""
    def __init__(self, targetmap, properties):
        targets = []
        for target, weight in targetmap.items():
            assert(hasattr(target, "velocity")), "{} doesn't have a velocity.".format(target)
            assert(hasattr(target, "mass")), "{} doesn't have a mass.".format(target)
            targets.append(target)
        self.init_properties(properties)
        self.efficiency = self._calc_efficiency(targetmap) # tells you how much power goes out compared to in
        self.targetmap = targetmap

    def distribute_energy(self, targetmap, joules):
        """Distributes the energy between the targets based on their weighting"""
        for target, weight in targetmap.items():
            self.inject_energy(target, joules * weight)

    def _calc_efficiency(self, targetmap):
        eff = 0
        for value in targetmap.values():
            eff += value
        return eff

    def component(self, item):
        if item in targetmap.keys():
            power_ratio = targetmap.get(item)
            return self.power_level * power_ratio
        else:
            warning("No component found for {} in {}".format(item, self))
            return 0

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


if __name__ == '__main__':
    # no arguments yet
    try:
        print("No main method here")
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


