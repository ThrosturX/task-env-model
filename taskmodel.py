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
    """Implements syntactic sugar allowing easy object manipulation"""
    def __add__(self, other):
        return self.value + other

    def __radd__(self, other):
        return self.value + other

    def __sub__(self, other):
        return self.value - other

    def __rsub__(self, other):
        return - self.value + other

    def __mul__(self, other):
        return self.value * other

    def __rmul__(self, other):
        return self.value * other

    def __lt__(self, other):
        return self.value < other

    def __gt__(self, other):
        return self.value > other

    def __str__(self):
        return "[{}]".format(self.value)

    def __float__(self):
        return float(self.value)

    def get_name(self):
        try:
            return self.name
        except AttributeError:
            return repr(self)

class UnboundedTaskEnvironmentObject(AbstractVariable):
    """'Physical' objects, 'slider'
    This class encapsulates the basic physics behind the objects
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
#           warning("{} is locked".format(self.name))
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
        sign_p = 1 if power >= 0 else -1
        sign_v = 1 if self.velocity >= 0 else -1
        F_comp_grav = - self.mass * self.gravity * math.sin(self.angle)
        F_fric = self.mass * self.gravity * math.cos(self.angle) * self.friction_kinetic

        # Branch based on velocity to avoid div/0 
        if self.velocity == 0:
            # Limited approximation method.
            # Calculate F_static in newtons and check if F_power is higher, if not, nothing happens (velocity stays at zero)
            # also account for gravity
            # Formulas:
            # F_power = a_power * m
            # a_power = sqrt(P/2mt)
            # F_static = mg cos(angle) * mu_static
            ###
            F_static = self.mass * self.gravity * math.cos(self.angle) * self.friction_static
            a_power = math.sqrt(abs(power)/(2 * self.mass * delta_time)) * sign_p
            a_movement = a_power - math.sin(self.angle) * self.gravity # gravity "goes the wrong way"
            F_movement = a_movement * self.mass
            # IF F_power > F_static THEN F_tot = F_power - F_net (F_net is the force due to friction and gravity)
            if abs(F_movement) > abs(F_static):
                # NOW a = F_tot / m and v = a * dt
                sign_f = 1 if F_movement >= 0 else -1
                F_tot = sign_f * (abs(F_movement) - abs(F_fric))
                assert(F_tot * sign_f > 0), "Dealing with negative forces? Then there's probably a bug in here!"
                acceleration = (F_tot / self.mass) 
                velocity = acceleration * delta_time
            else:
                velocity = 0
        else:
            # Friction acts in the opposite direction of velocity
            opps = -1 if power < 0 and self.velocity < 0 else 1
            F_fric = - F_fric * sign_v 
            # m dv / dt = P/v+F => dv = (P/v+F)dt / m
            # also: dv = ( P/v + F0 + F1 + ... + Fn) dt / m
            # where P/v is the force from power (negative if signs differ)
            F_power = abs(power / self.velocity) * sign_p
            F_movement = F_power + F_comp_grav
            dv = (F_movement + F_fric) * delta_time / float(self.mass)
            velocity = self.velocity + dv 
#           print("Fp: {:.2f} Fg: {:.2f} Fm: {:.2f} (v0, v1): {:.5f}, {:.5f}".format(F_power, F_comp_grav, F_movement, self.velocity, velocity))
            # If there exists a point 0 < c < dt where v(c) == 0, then we need to go back to
            # the case where velocity is zero.                      NOTE NOTE NOTE NOTE NOTE
            # We can either stop the item for this time slice,      NOTE NOTE NOTE NOTE NOTE
            # or split the calculation up further                   NOTE NOTE NOTE NOTE NOTE
            # by calculating the exact time it stops and continuing from there,    NOTE NOTE
            # but I leave this as future work
            if velocity > 0 and sign_v < 0 \
            or velocity < 0 and sign_v > 0: # if the sign changed, then v(c) == 0 exists for 0 < c < dt
                velocity = +0.0

        return velocity

    def lock(self):
        self.locked = True

    def unlock(self):
        self.locked = False

    # TODO: COULD USE TESTING
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
        # TODO: COULD USE TESTING
        if abs(F_tot) < abs(F_fric): # force can't overcome friction
            if self.velocity == 0:
                acceleration = 0
            elif self.velocity > 0:
                acceleration = -F_fric + F_tot / self.mass
            elif self.velocity < 0:
                acceleration = +F_fric + F_tot / self.mass
        return acceleration

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

    # If there is no kinetic friction, the energy required is at the limit of zero and time depends on how much energy you put in
    def calc_drift(self, goal, epsilon=0):
        """Calculate the kinetic energy, initial velocity and time required for the item to drift into the goal"""
        distance = abs(goal - self.value) - epsilon
                 # F_Grav * angle * mu ## we assume no static friction for optimization
        F_fric   = - (self.mass * self.gravity) * math.cos(self.angle) * self.friction_kinetic
        F_move   = - (self.mass * self.gravity) * math.sin(self.angle) # movement due to gravity (if any)
        F_tot = F_fric + F_move
        accel = F_tot / self.mass
        # calculate the energy required to accelerate the item to a velocity that will drift into the final position
        work = F_tot * distance
        drift_kin = abs(work) # 0.5 * self.mass * (drift_vel ** 2)
        drift_vel = math.sqrt((2 * drift_kin)/self.mass)
        drift_time = abs(drift_vel / accel)
        return (drift_kin, drift_vel, drift_time)

    def calc_min_energy(self, goal, max_time, epsilon=0):
        """Calculate the energy required to accelerate the object to a velocity that will drift into the final position"""
        drift_kin, drift_vel, drift_time = self.calc_drift(goal, epsilon)
        # if it takes too long, increase the energy required appropriately
        if drift_time > max_time and max_time > 0:
            distance = abs(goal - self.value) - epsilon
            F_fric   = - (self.mass * self.gravity) * math.cos(self.angle) * self.friction_kinetic
            F_move   = - (self.mass * self.gravity) * math.sin(self.angle) # movement due to gravity (if any)
            F_tot = F_fric + F_move
            accel = F_fric / self.mass
            min_vel = distance / max_time - accel * max_time / 2
            E_push = self.mass * (min_vel ** 2) / 2
            # now we need to calculate the cost of stopping it
            vel_time_t = min_vel + accel * max_time
            E_stop = abs(self.mass * (vel_time_t ** 2) / 2)

            return E_push + E_stop, (E_push, E_stop)
        return drift_kin, (drift_kin, 0)

    # TODO: refactor away!?
    def calc_min_time(self, max_joules):
        """Calculate the number of seconds it will take to move the object to the final position given a limited amount of energy""" 
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

    def calc_min_time_e(self, goal, epsilon=0):
        displacement = abs(goal - self.value) - epsilon
        direction = 1 if goal > self.value else -1
        # affectors decide how fast it can move...
        total_max_power = 0
        for motor in self.affectors:
            power = motor.max_power
            if direction == -1: # we need to reverse
                if motor.reversible:
                    power = motor.max_power * motor.reverse_power_ratio
                else: # this motor is useless right now
                    power = 0
            total_max_power += abs(power)
        if total_max_power == 0:
            return 0 # we can't affect this
        # Now we know how much power we can apply
        # We use s = sqrt(8Pt^3 / 9m) => t = cube(9ms^2/8P)
        min_time = ((9 * self.mass * (displacement ** 2))/(8 * total_max_power)) ** (1.0/3.0)
        return min_time, total_max_power

    def get_profile(self, goal, epsilon=0):
        min_e, _, min_e_time = self.calc_drift(goal, epsilon=epsilon)
        min_t, power = self.calc_min_time_e(goal, epsilon=epsilon)
        min_t_e, _ = self.calc_min_energy(goal, min_t, epsilon=epsilon)
        profile = {}
        profile['min_energy'] = (min_e, min_e_time)
        profile['min_time'] = (min_t_e, min_t)
        profile['self'] = self
        profile['goal'] = goal
        times = numpy.arange(min_t, min_e_time, 0.1)
        energies = [self.calc_min_energy(goal, time)[0] for time in times]
        profile['curve'] = times, energies
        return profile

class TaskEnvironmentObject(UnboundedTaskEnvironmentObject):
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
    """Transition function allowing mutations to objects"""
    def __init__(self, affected_objects, transition_function):
        self.affected_objects = affected_objects
        self.transition = transition_function
        def precondition(): return True
        self.precondition = precondition

    def set_precondition(self, precondition):
        self.precondition = precondition

    def apply_transition(self, *args):
        """Apply a transition function to objects and return the resulting object set"""
        if not self.precondition():
            return None
        if self.affected_objects: # affected_objects get unpacked
            new_objects = self.transition(*self.affected_objects, *args)
        else:
            assert(args), "This transition requires arguments\n({})".format(self.transition)
            new_objects = self.transition(*args)
        return new_objects

class TaskEnvironmentGoal(AbstractVariable):
    def __init__(self, target, goal_value, goal_epsilon):
        # maybe check the types here
        self.target = target
        self.value = goal_value
        self.epsilon = goal_epsilon
        self.satisfied = False
        self.prerequisites = []

    def add_prerequisite(self, goal):
        self.prerequisites.append(goal)

    def assess(self):
        if self.current_condition():
            for prereq in self.prerequisites:
                if not prereq.current_condition():
                    return
            self.satisfied = True

    def current_condition(self):
        if self.target < self.value + self.epsilon \
        and self.target > self.value - self.epsilon:
            return True

    # Recursively reset this goal and prerequisites 
    def reset(self):
        self.satisfied = False
        for prereq in self.prerequisites:
            prereq.reset()

    def __bool__(self):
        self.assess()
        return self.satisfied

class TaskEnvironmentSystem(object):
    """Class to encapsulate the behavior of objects. Simplifies interaction.
    Systems can have objects, transitions, motors, sensors and other systems.
    """
    # Beware this constructor, it seems like sometimes I have to specify explicitly, otherwise they take values from memory?
    def __init__(self, objects=[], transitions=[], motors=[], sensors=[], systems=[]):
        if not hasattr(objects, '__iter__'):
            objects = [objects]
        if not hasattr(transitions, '__iter__'):
            transitions = [transitions]
        if not hasattr(motors, '__iter__'):
            motors = [motors]
        if not hasattr(sensors, '__iter__'):
            sensors = [sensors]
        if not hasattr(systems, '__iter__'):
            systems = [systems]
        self.objects = objects
        self.transitions = transitions
        self.motors = motors
        self.sensors = sensors
        self.systems = systems

    def natural_transition(self, delta_time):
        for transition in self.transitions:
            transition.apply_transition(delta_time)

    def satisfies(self, solution):
        result = solution(*self.objects)
        return result

    def all_objects(self):
        if not self.systems:
            return self.objects
        all_objects = set(self.objects.copy())
        for system in self.systems:
            all_objects = all_objects.union(system.all_objects())
        return list(all_objects)

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

    def lock_objects(self):
        for object in self.objects:
            object.lock()

    def unlock_objects(self):
        for object in self.objects:
            object.unlock()

class TaskEnvironmentModel(object):
    """Represent task environment models as E = {V,T}
    With some abstractions.
    """
    def __init__(self, environment, solution, max_time=0, max_energy=0):
        self.environment = environment  # should be a system
        self.solution = solution        # should be a list of goals
        self.clock = 0
        self.dt = 0.001
        self.solution_score = 0.0 # the model starts as 'unsolved' with respect to any solutions
        self.max_time = max_time
        self.max_energy = max_energy

    def all_objects(self):
        all_vars = set()
        for var in self.environment.all_objects():
            all_vars.add(var)
        return list(all_vars)

    def tick(self, delta_time):
        """Affect every object with the natural change caused by delta_time seconds elapsing"""
        time_passed = 0
        while time_passed < delta_time:
            time_passed += self.dt
            # let all systems tick
            for system in self.environment.all_systems():
                system.natural_transition(float(self.dt))
            # let all objects tick
            for obj in self.all_objects():
                obj.natural_transition(float(self.dt))
            self.check_for_solutions()
        for motor in self.motors():
            joules = abs(motor.power_level + motor.wasted_power) * time_passed
            motor.usage += joules
        self.clock += time_passed

    # a solution is set of booleans representing achieved goals
    # for now solutions are simple booleans
    def check_for_solutions(self):
        if self.solved():
            self.solution_score = 1.0

    def motors(self):
        return self.environment.all_motors()

    def sensors(self):
        return self.environment.all_sensors()

    def solved(self):
        for goal in self.solution:
            goal.assess()
            if not goal.satisfied:
                return False
        return True

    def goal_vars(self):
        g_vars = []
        for goal in self.solution:
            element = goal.target, goal.value, goal.epsilon
            g_vars.append(element)
        return g_vars

    def reset(self):
        # randomize goal vars
        for var, g, d in self.goal_vars():
            lb = 0
            ub = 1
            if hasattr(var, 'lower_bound'):
                lb = var.lower_bound
            if hasattr(var, 'upper_bound'):
                ub = var.upper_bound
            val = random.uniform(lb, ub)
            var.value = val
            var.velocity = 0
            if var < g + d and var > g - d:
                var = lb
        # reset all goal states
        for goal in self.solution:
            goal.reset()
        # reset motors and energy expenditure
        for motor in self.motors():
            motor.usage = 0
            motor.power_level = 0
            motor.wasted_power = 0
        # reset clock
        self.clock = 0

    def energy_needed(self):
        """Calculate the necessary power to move every object into a goal position"""
        joule_total = 0
        for var, goal, epsilon in self.goal_vars():
            if var.friction_kinetic > 0:  # kinetic friction present, calculate drift energy
                E_min, _, _ = var.calc_drift(goal, epsilon)
                joules = E_min
            else: # There is no kinetic friction, so essentially this object is 'free' apart from the initial push and equal stop, which we won't count since it depends on a time resolution
                joules = 0 # We might want to add some "minimum power"
            # calculate energy gained (or saved) because of acceleration
            if var.calc_acceleration() != 0:
                # the object is (probably) moving away from its goal constantly,
                # so calculate the energy to move it directly to the goal
                a = var.calc_acceleration()
                d = abs(var - goal)
                # edge case #1: acceleration moves the value into the goal at some point in the future
                if a > 0 and var < goal \
                or a < 0 and var > goal:
                    joules = 0
                else:
                    # work = force x distance
                    work = abs(var.mass * a * d)
                    joules += work 
            joule_total += joules
        return joule_total

    def get_profiles(self):
        profiles = []
        for var, goal, epsilon in self.goal_vars():
            profile = var.get_profile(goal, epsilon=epsilon)
            profiles.append(profile)
        return profiles

    def calc_curve_time(self, low, high):
        times = numpy.arange(low, high, 0.1)
        energy = [0] * len(times)
        for x, time in enumerate(times):
            for var, goal, epsilon in self.goal_vars():
                energy[x] = energy[x] + var.calc_min_energy(goal, time, epsilon=epsilon)[0]
                print("{} {} {}".format(time, energy[x], var.calc_min_energy(goal, time, epsilon=epsilon)))
        return times, energy
    
    def get_profile(self):
        profile = {}
        me = 0
        mt = 0
        total_power = 0
        powers = []
        min_times = []
        min_e_times = []
        for var, goal, epsilon in self.goal_vars():
            min_e, vel, min_e_time = var.calc_drift(goal, epsilon=epsilon)
            mte, power = var.calc_min_time_e(goal, epsilon=epsilon)
#           ramp_time = ((vel ** 2) * var.mass) / (2 * power)
#           min_e_time += ramp_time
            min_e_times.append(min_e_time)
            min_times.append(mte)
            powers.append(power)
            me = me + min_e

        total_power = sum(powers)
        # Now choose the highest min_time and calculate min_time energy based on that     
        max_time = sum(min_times)
        for var, goal, epsilon in self.goal_vars():
            min_t_e, _ = var.calc_min_energy(goal, max_time, epsilon=epsilon)
            mt = mt + min_t_e

        me = me, max(min_e_times) # should not use a lot of energy, so will take longer
        mt = mt, max(min_times) # should be fast but use a lot of energy
        profile['min_energy'] = me
        profile['min_time'] = mt
        curve = self.calc_curve_time(mt[1], me[1])
        profile['curve'] = curve
        profile['power'] = total_power
        profile['max_time'] = self.max_time
        profile['max_energy'] = self.max_energy
#       # NOTE: Just return the most greedy profile
#       max_energy = 0
#       profile = None
#       for pp in self.get_profiles():
#           max_energy = max(max_energy, pp['min_energy'][0])
#           if pp['min_energy'][0] == max_energy:
#               profile = pp
        return profile

    def used_energy(self):
        return sum([motor.usage for motor in self.motors()])

    def failed(self):
        fail_time = self.clock > self.max_time and self.max_time > 0
        fail_energy = self.used_energy() > self.max_energy and self.max_energy > 0 
        return fail_time or fail_energy

class Motor(object): # 'Actuator'
    """A class to affect TaskEnvironmentObjects"""
    def __init__(self, target, properties):
        assert(hasattr(target, "velocity")), "{} doesn't have a velocity.".format(target)
        assert(hasattr(target, "mass")), "{} doesn't have a mass.".format(target)
        self.target = target
        self.init_properties(properties)
        self.power_level = 0
        self.wasted_power = 0
        self.usage = 0
        target.affectors.append(self)

    def init_properties(self, properties={}):
        """Motors have some default properties that can be overwritten"""
        self.max_power = properties.get('max_power', 100)
        self.reversible = properties.get('reversible', True)
        self.reverse_power_ratio = properties.get('reverse_power_ratio', 1.0)

    # Note: watts express the rate of energy transfer with respect to time
    def activate(self, watts):
        self.power_level = watts
        # conditional checks regarding whether it can be activated in this way
        if watts < 0 and not self.reversible:
            warning("Attempting to reverse a unidirectional motor")
            self.power_level = 0
        if watts > self.max_power:
            self.power_level = self.max_power
            warning("Attempting to activate a {} W motor with {} W".format(self.max_power, watts))
        if watts < 0 and watts > self.reverse_power_ratio * self.max_power:
            warning("Reversing a {} W motor at {} W with a ratio of {} (max reversal power is {})".format(self.max_power, watts, self.reverse_power_ratio, self.reverse_power_ratio * self.max_power))
            self.power_level = self.reverse_power_ratio * self.max_power
        self.wasted_power = max(0, abs(watts - self.power_level))

    def component(self, item):
        if item == self.target:
            return self.power_level
        warning("Attempting to find the component of a motor that doesn't target {}".format(item))
        return 0

class MultiMotor(Motor):
    """A class that extends Motors to allow modifications to multiple TaskEnvironmentObjects"""
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
    """A class that reads object values and rounds them after optionally distorting them
    Distortion is the maximum amount that the sensor reading can deviate from the correct value
    """
    def __init__(self, observed_object, rounding_digits=0, distortion=0):
        self.target = observed_object
        self.n_digits = rounding_digits
        self.distortion = distortion

    def read(self):
        """Return a perceived value from the sensor"""
        if type(self.target) == Sensor:
            value = self.target.read()
        else: 
            value = self.target.value
        value = value + random.uniform(-self.distortion, self.distortion)
        return round(value, self.n_digits)


def temp_test():
    import samples
    t, g = samples.sample_system_1D_plotter()
    vs = t.all_objects()
    for vv in vs:
        if not hasattr(vv, 'name'):
            continue
        if vv.name == 'position':
            v = vv
            break
    assert(v), 'no v'
    v.value = 20
    print(v.affectors)
    mt, p = v.calc_min_time_e(g.value)
    me, _, me_t = v.calc_drift(g.value)
#   print("min time: {}".format(mt))
#   print("min time energy: {} (power * min time)".format(mt*p))
#   print("min energy: {}, min time: {}".format(me, me_t))
    profile = v.get_profile(g)
    print("profile: {}".format(profile))
    m = t.motors()[0]
    m.activate(100)
    t.tick(profile['min_time'][1])
    m.activate(0)
    print(t.solved())
    print(v.value)
    t.reset()
    v.value = 20
    m.activate(100)
    seconds = profile['min_energy'][0] / 100
    print("activating 100W for {} seconds".format(seconds))
    t.tick(seconds)
    m.activate(0)
    part2 = profile['min_energy'][1] - seconds
    print("ticking {} more seconds".format(part2))
    t.tick(part2)
    print(t.solved())
    print(v.value)


if __name__ == '__main__':
    # no arguments yet
    try:
        print("No main method here")
        temp_test()
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


