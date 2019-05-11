from data_collector import TrajectoryCollector
import numpy as np

# intialize the trajectory collector, containing all the flat outputs
data = TrajectoryCollector('trajectory_generation.mat')
# call the flat outputs at a specific time
flat_outputs = data.interpolation_linear(4)
# display the flat outputs
np.set_printoptions(precision=4)
print('load position',flat_outputs['xl'])
print('load velocity',flat_outputs['vl'])
print('load acceleration',flat_outputs['al'])
print('load 1st derivative of acceleration',flat_outputs['dal'])
print('load 2nd derivative of acceleration',flat_outputs['d2al'])
print('load 3rd derivative of acceleration',flat_outputs['d3al'])
print('load 4th derivative of acceleration',flat_outputs['d4al'])
