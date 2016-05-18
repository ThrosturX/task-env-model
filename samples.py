import taskmodel
from taskmodel import *

taskmodel.DEBUG = True

def inform(env):
    for variable in env.all_variables():
        print("{}: {} (v: {})".format(variable, variable.value, variable.velocity))

# SAMPLE 1:
# In this sample, the environment is a single 'position' variable and
# the task is to set this variable to a goal value
def sample_system_1D_plotter(max_power=100, default_start=50, default_delta=50):
    position = TaskEnvironmentObject(default_start, default_delta)
    position.set_bounds(0, 150)
    position.gravity = 10
    position.friction_kinetic = 0.2
    position.friction_static = 0.3
    position.mass = 5 
    position.name = 'position'  # syntactic sugar, just gives a label to the variable for printing
    motor_properties = {'max_power': max_power, 'reversible': True, 'reverse_power_ratio': 1.0}
    motor = Motor(position, motor_properties)
    sensor = Sensor(position, rounding_digits=1, distortion=0.5)
    hammer = TaskEnvironmentSystem([position], motors=motor, sensors=sensor)
    # The 'ENVIRONMENT' is the 'hammer' system in this case 
    # The 'TASK' is to set the 'hammer' system's internal variable, 'position' to some defined 'goal'
    goal_position = UnboundedTaskEnvironmentObject(100, 0)
    goal_epsilon = UnboundedTaskEnvironmentObject(5, 0)
    goal_vars = [goal_position, goal_epsilon]
    solution = TaskEnvironmentGoal(position, goal_position, goal_epsilon)
    sol_system = TaskEnvironmentSystem(goal_vars, systems=hammer)
    task = TaskEnvironmentModel(sol_system, [solution], max_time=60, max_energy=5000)
    return task, solution

def sample_system_1Db_plotter(max_power=100, default_start=50, default_delta=50):
    # Variables
    position = TaskEnvironmentObject(default_start, default_delta)
    position.set_bounds(0, 150)
    position.gravity = 10
    position.friction_kinetic = 0.2
    position.friction_static = 0.3
    position.mass = 5 
    position.name = 'position'   # syntactic sugar, just gives a label to the variable for printing
    # whack hammer variable
    whack_it = TaskEnvironmentObject(0, 0)
    whack_it.set_bounds(0, 1)
    whack_it.gravity = 10
    whack_it.angle = math.pi / 2 # this makes it require power to keep it at '1'
    whack_it.friction_kinetic = 0.50
    whack_it.friction_static = 0.55
    whack_it.name = 'hammer_down'
    whack_it.mass = 1 # not necessary (default value)
    # motors
    motor_properties = {'max_power': max_power, 'reversible': True, 'reverse_power_ratio': 1.0}
    position_motor = Motor(position, motor_properties)
    whack_motor = Motor(whack_it, {'max_power':5, 'reversible': False})
    motors = [position_motor, whack_motor]
    # sensors
    position_sensor = Sensor(position, rounding_digits=1, distortion=0.5)
    whack_sensor = Sensor(whack_it, rounding_digits=0, distortion=0.1)
    sensors = [position_sensor, whack_sensor]
    # create the environment system
    variables = [position, whack_it]
    hammer = TaskEnvironmentSystem(variables, motors=motors, sensors=sensors)
    # goal setup
    goal_position = UnboundedTaskEnvironmentObject(100, 0)
    goal_epsilon = UnboundedTaskEnvironmentObject(5, 0)
    goal_whack = UnboundedTaskEnvironmentObject(1, 0)
    goal_whack_d = UnboundedTaskEnvironmentObject(0.1, 0)
    goal_vars = [goal_position, goal_epsilon, goal_whack, goal_whack_d]
    # goal 1 prerequisite: in right position
    correct_position = TaskEnvironmentGoal(position, goal_position, goal_epsilon)
    # goal 1: hammer whacked (while in right position)
    mole_whacked = TaskEnvironmentGoal(whack_it, goal_whack, goal_whack_d)
    mole_whacked.add_prerequisite(correct_position)
    # create the task+environment system (the task has been encoded into goals)
    sol_system = TaskEnvironmentSystem(goal_vars, systems=hammer)
    # create the task environment model with accompanying goals
    task = TaskEnvironmentModel(sol_system, [correct_position, mole_whacked], max_time=120, max_energy=10000)
    return task, mole_whacked

def sample_system_1Db_plotter2(max_power=100, default_start=50, default_delta=50): # lock position while whack_it
    # Variables
    position = TaskEnvironmentObject(default_start, default_delta)
    position.set_bounds(0, 150)
    position.gravity = 10
    position.friction_kinetic = 0.2
    position.friction_static = 0.3
    position.mass = 5 
    position.name = 'position'  # syntactic sugar, just gives a label to the variable for printing
    # whack hammer variable
    whack_it = TaskEnvironmentObject(0, 0)
    whack_it.set_bounds(0, 1)
    whack_it.gravity = 10
    whack_it.angle = math.pi / 2 # this makes it require power to keep it at '1'
    whack_it.friction_kinetic = 0.50
    whack_it.friction_static = 0.55
    whack_it.name = 'hammer_down'
    whack_it.mass = 1 # not necessary (default value)
    # motors
    motor_properties = {'max_power': max_power, 'reversible': True, 'reverse_power_ratio': 1.0}
    position_motor = Motor(position, motor_properties)
    whack_motor = Motor(whack_it, {'max_power':5, 'reversible': False})
    motors = [position_motor, whack_motor]
    # sensors
    position_sensor = Sensor(position, rounding_digits=1, distortion=0.5)
    whack_sensor = Sensor(whack_it, rounding_digits=0, distortion=0.1)
    sensors = [position_sensor, whack_sensor]
    # create the environment system
    variables = [position, whack_it]
    def func_lock_pos(pos, whack, delta_time):
        if whack > 0.5:
            pos.lock()
        else:
            pos.unlock()
    lock_pos = TaskEnvironmentTransition(variables, func_lock_pos)
    hammer = TaskEnvironmentSystem(variables, motors=motors, sensors=sensors, transitions=[lock_pos])
    # goal setup
    goal_position = UnboundedTaskEnvironmentObject(100, 0)
    goal_epsilon = UnboundedTaskEnvironmentObject(5, 0)
    goal_whack = UnboundedTaskEnvironmentObject(1, 0)
    goal_whack_d = UnboundedTaskEnvironmentObject(0.1, 0)
    goal_vars = [goal_position, goal_epsilon, goal_whack, goal_whack_d]
    # goal 1 prerequisite: in right position
    correct_position = TaskEnvironmentGoal(position, goal_position, goal_epsilon)
    # goal 1: hammer whacked (while in right position)
    mole_whacked = TaskEnvironmentGoal(whack_it, goal_whack, goal_whack_d)
    mole_whacked.add_prerequisite(correct_position)
    # create the task+environment system (the task has been encoded into goals)
    sol_system = TaskEnvironmentSystem(goal_vars, systems=hammer)
    # create the task environment model with accompanying goals
    task = TaskEnvironmentModel(sol_system, [correct_position, mole_whacked], max_time=60, max_energy=10000)
    return task, mole_whacked

def sample_system_2D_plotter():
    # Variables
    pos_x = TaskEnvironmentObject(30, 20)
    pos_y = TaskEnvironmentObject(30, 20)
    plotter = TaskEnvironmentObject(0, 0)
    position = [pos_x, pos_y] 
    # configure variables and create motors
    motors = []
    for component in position: # both of our components are orthogonal to 'up'
        # configure component
        component.set_bounds(0, 150)
        component.gravity = 10
        component.friction_kinetic = 0.2
        component.friction_static = 0.3
        component.mass = 5
        component.name = 'pos_comp_' + str(position.index(component))
        # create a motor for this component
        motor_properties = {'max_power': 100, 'reversible': True, 'reverse_power_ratio': 1.0}
        current_motor = Motor(component, motor_properties)
        motors.append(current_motor)
    # create a variable for 'whacking hammer is down'
    plotter.set_bounds(0, 1)
    plotter.gravity = 10
    plotter.angle = math.pi / 2 # 1 becomes 'up', which is fine since 'gravity' points down (but we model it as hand wants to stay raised instead)
    plotter.friction_kinetic = 0.50
    plotter.friction_static = 0.90
    plotter.name = 'plotter'
    whack_motor = Motor(plotter, {'max_power':5, 'reversible': False})
    motors.append(whack_motor)
    # motors should now contain all motors (direct access to each variable)
    sensor_x = Sensor(pos_x, rounding_digits=2, distortion=2)
    sensor_y = Sensor(pos_y, rounding_digits=2, distortion=2)
    sensor_plot = Sensor(plotter, rounding_digits=0, distortion=0.1)
    sensors = [sensor_x, sensor_y, sensor_plot]
    env_vars = [pos_x, pos_y, plotter]
    plotting_system = TaskEnvironmentSystem(env_vars, motors=motors, sensors=sensors)
    # goals
    goal_pos_x = UnboundedTaskEnvironmentObject(100, 0)
    goal_pos_y = UnboundedTaskEnvironmentObject(100, 0)
    goal_plot = UnboundedTaskEnvironmentObject(1, 0)
    epsilon_pos = UnboundedTaskEnvironmentObject(5, 0)
    epsilon_plot = UnboundedTaskEnvironmentObject(0.1, 0)
    goal_vars = [goal_pos_x, goal_pos_y, goal_plot, epsilon_pos, epsilon_plot]
    # correct position goals
    correct_x = TaskEnvironmentGoal(pos_x, goal_pos_x, epsilon_pos)
    correct_y = TaskEnvironmentGoal(pos_y, goal_pos_y, epsilon_pos)
    # main goal: plot/whack the right place
    correct_plot = TaskEnvironmentGoal(plotter, goal_plot, epsilon_plot)
    correct_plot.add_prerequisite(correct_x)
    correct_plot.add_prerequisite(correct_y)
    sol_system = TaskEnvironmentSystem(goal_vars, systems=plotting_system)
    task = TaskEnvironmentModel(sol_system, [correct_plot], max_time=120, max_energy=20000)
    return task, correct_plot

def sample_rotating_motor():
    pos_x = TaskEnvironmentObject(30, 20)
    pos_y = TaskEnvironmentObject(30, 20)
    pos_x.name = 'px'
    pos_y.name = 'py'
    pos_x.set_bounds(0, 150)
    pos_y.set_bounds(0, 150)
    low = -math.pi
    high = math.pi
    length = high - low
    angle = TaskEnvironmentObject(low + length/2, length/2)
    angle.set_bounds(low, high)
    # create some sensors
    sensor_angle = Sensor(angle, rounding_digits=0, distortion=0.1)
    sensor_x = Sensor(pos_x, rounding_digits=2, distortion=2)
    sensor_y = Sensor(pos_y, rounding_digits=2, distortion=2)
    # set up the rotating motors
    motor_props = {'max_power':10, 'reversible': True}
    main_power = TaskEnvironmentObject(0, 0) # this 'useless' variable gives us a handle to the motor in the transition
    main_power.set_bounds(-100, 100)

    motor_x = Motor(pos_x, motor_props)
    motor_y = Motor(pos_y, motor_props)

    def rotating_motor_transition(pos_x, pos_y, main_power, angle, delta_time):
        # check how much power main motor is using
        total_power = main_power.affectors[0].power_level
        main_power.value = total_power
        main_power.velocity = 0
        # calculate x and y components based on angle
        power_x = math.cos(angle.value) * total_power
        power_y = math.sin(angle.value) * total_power
        motor_x.activate(power_x)
        motor_y.activate(power_y)
        motor_x.usage = 0   # subtract previous energy use on this motor
        motor_y.usage = 0   # since the energy comes from another motor that we count
    transition = TaskEnvironmentTransition([pos_x, pos_y, main_power, angle], rotating_motor_transition)
    # the two motors that we actually expose:
    rotator = Motor(angle, motor_props)
    main_motor = Motor(main_power, motor_props)

    # the system encapsulating this behavior
    rotating_system = TaskEnvironmentSystem([pos_x, pos_y, angle, main_power], transitions=[transition], motors=[rotator, main_motor, motor_x, motor_y], sensors=[sensor_angle, sensor_x, sensor_y])
    # now we can create some task, or use this system to create a more complicated system...
    goal_pos_x = UnboundedTaskEnvironmentObject(100, 0)
    goal_pos_y = UnboundedTaskEnvironmentObject(100, 0)
    epsilon = UnboundedTaskEnvironmentObject(9, 0)
    gx = TaskEnvironmentGoal(pos_x, goal_pos_x, epsilon)
    gy = TaskEnvironmentGoal(pos_y, goal_pos_y, epsilon)
    solution = TaskEnvironmentGoal(1, 1, 1)
    solution.add_prerequisite(gx)
    solution.add_prerequisite(gy)
    task = TaskEnvironmentModel(rotating_system, [solution])
    return rotating_system, task

def sample_N_task(N, delta=2):
    env = TaskEnvironmentSystem(objects=[], sensors=[], systems=[])
    sol = []
    hidden_motor_system = TaskEnvironmentSystem(objects=[], transitions=[], motors=[], sensors=[], systems=[])
    for x in range(N):
        obj = TaskEnvironmentObject(3, delta)
        obj.name = 'obj_' + str(x)
        obj.set_bounds(0, 10)
        obj.gravity = 10
        obj.friction_kinetic = 0.2
        obj.friction_static = 0.3
        sensor = Sensor(obj, 0, 0.05)
        goal = TaskEnvironmentGoal(obj, 10, 0.5)
        motor = Motor(obj, {'max_power':1, 'reversible': False}) # hidden motor!
        hidden_motor_system.motors.append(motor)
        env.objects.append(obj)
        env.sensors.append(sensor)
        sol.append(goal)
    env.systems.append(hidden_motor_system)
    active_dimension = TaskEnvironmentObject(0, 0)
    active_dimension.mass = 0.01
    active_dimension.name = 'enumerator'
    active_dimension.set_bounds(0, N-1)
    activation_power = TaskEnvironmentObject(0, 0)
    activation_power.name = 'power_level'
    activation_power.set_bounds(0, 1)
    s_dim = Sensor(active_dimension, 0, 0)
    s_pow = Sensor(activation_power, 0, 0)
    selector_motor = Motor(active_dimension, {'max_power':0.1, 'reversible': True})
    activator_motor = Motor(activation_power, {'max_power':1, 'reversible': False})

    def func_transition(active_dimension, activation_power, objects, delta_time):
        main_motor = activation_power.affectors[0]
        power = main_motor.power_level
        activation_power.value = power
        objs_named = True
        for obj in objects:
            if not hasattr(obj, 'name'):
                objs_named = False
                break
        if objs_named:
            objects.sort(key=lambda x: x.name) # note: in-place sort
        selection = objects[int(active_dimension.value)]
        target_motor = selection.affectors[0]
        target_motor.activate(power) # this is a hidden motor, which gets counted with the hidden_motor_system
#       main_motor.power_level = 0
        main_motor.wasted_power = 0
        main_motor.usage = 0

    affected_objs = [active_dimension, activation_power, env.objects]
    transition = TaskEnvironmentTransition(affected_objs, func_transition)
    control_system = TaskEnvironmentSystem([active_dimension, activation_power], transitions=[transition], motors=[selector_motor, activator_motor], sensors=[s_dim, s_pow], systems=[])
    env.systems.append(control_system)
    task = TaskEnvironmentModel(env, sol, max_time=20, max_energy=200)
    min_energy = task.energy_needed()
    maxen = min_energy * 3
    task.max_energy = maxen
    task.max_time = maxen / 9
    return task, sol



