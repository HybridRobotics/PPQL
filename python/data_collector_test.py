from data_collector import TrajectoryCollector
import numpy as np
import time

# intialize the trajectory collector, containing all the flat outputs
data = TrajectoryCollector('../data/testMoving.mat')
np.set_printoptions(precision=4)
# call the flat outputs at a specific timestamp 

# TEST 1: not start yet, use the start_state
time_start = time.clock()
timestamp = -1
flat_outputs = data.interpolation_linear(timestamp)
print('Read data at timestamp t = ', timestamp)
# display the flat outputs
print('timestamp',flat_outputs['time'])
print('status',flat_outputs['status'])
print('quadrotor position',flat_outputs['xQ'])
print('load position',flat_outputs['xL'])
print('load velocity',flat_outputs['vL'])
print('load acceleration',flat_outputs['aL'])
print('load 1st derivative of acceleration',flat_outputs['daL'])
print('load 2nd derivative of acceleration',flat_outputs['d2aL'])
print('load 3rd derivative of acceleration',flat_outputs['d3aL'])
print('load 4th derivative of acceleration',flat_outputs['d4aL'])
time_elapsed = (time.clock() - time_start)
print('computational time for getting values', time_elapsed)

# TEST 2: during the path planning, get values from interpolation
time_start = time.clock()
timestamp = 3
flat_outputs = data.interpolation_linear(timestamp)
print('Read data at timestamp t = ', timestamp)
# display the flat outputs
print('timestamp',flat_outputs['time'])
print('status',flat_outputs['status'])
print('quadrotor position',flat_outputs['xQ'])
print('load position',flat_outputs['xL'])
print('load velocity',flat_outputs['vL'])
print('load acceleration',flat_outputs['aL'])
print('load 1st derivative of acceleration',flat_outputs['daL'])
print('load 2nd derivative of acceleration',flat_outputs['d2aL'])
print('load 3rd derivative of acceleration',flat_outputs['d3aL'])
print('load 4th derivative of acceleration',flat_outputs['d4aL'])
time_elapsed = (time.clock() - time_start)
print('computational time for getting values', time_elapsed)

# TEST 3: not start yet, use the end_state
time_start = time.clock()
timestamp = 10
flat_outputs = data.interpolation_linear(timestamp)
print('Read data at timestamp t = ', timestamp)
# display the flat outputs
print('timestamp',flat_outputs['time'])
print('status',flat_outputs['status'])
print('quadrotor position',flat_outputs['xQ'])
print('load position',flat_outputs['xL'])
print('load velocity',flat_outputs['vL'])
print('load acceleration',flat_outputs['aL'])
print('load 1st derivative of acceleration',flat_outputs['daL'])
print('load 2nd derivative of acceleration',flat_outputs['d2aL'])
print('load 3rd derivative of acceleration',flat_outputs['d3aL'])
print('load 4th derivative of acceleration',flat_outputs['d4aL'])
time_elapsed = (time.clock() - time_start)
print('computational time for getting values', time_elapsed)