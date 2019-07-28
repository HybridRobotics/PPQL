import scipy.io as sio
from scipy import interpolate

class TrajectoryCollector:
	def __init__(self,filename):
		data = sio.loadmat('../data/'+filename)
		self.time_list = data['time_list']
		self.status_list = data['status_list']
		self.xQ_list = data['xQ_list']
		self.xL_list = data['xL_list']
		self.vL_list = data['vL_list']
		self.aL_list = data['aL_list']
		self.daL_list = data['daL_list']
		self.d2aL_list = data['d2aL_list']
		self.d3aL_list = data['d3aL_list']
		self.d4aL_list = data['d4aL_list']

	# get the size of trajectory
	def get_trajectory_size(self):
		return len(self.time_list[0,:])

	# get the index most closed to the given time
	def get_index(self, time):
		if time <= self.time_list[0, 0]:
			# trajectory not started
			return -1
		elif time >= self.time_list[0, -1]:
			# trajectory finished
			return self.get_trajectory_size()-1
		else:
			# BFS to get the index
			left = 0
			right = self.get_trajectory_size()-1
			while (right > left + 1):
				mid = (int) (left+right)/2
				if time > self.time_list[0, mid]:
					left = mid
				elif time < self.time_list[0, mid]:
					right = mid
				else:
					return self.time_list[0, mid]
			return left

	def interpolation_linear(self, time):
		ind = self.get_index(time)
		if ind == -1:
			time = self.time_list[0, 0]
			status = self.status_list[0, 0]
			xQ = self.xQ_list[:, 0]
			xL = self.xL_list[:, 0]
			vL = self.vL_list[:, 0]
			aL = self.aL_list[:, 0]
			daL = self.daL_list[:, 0]
			d2aL = self.d2aL_list[:, 0]
			d3aL = self.d3aL_list[:, 0]
			d4aL = self.d4aL_list[:, 0]
		elif ind == self.get_trajectory_size() - 1:
			# use last pose of the trajectory
			time = self.time_list[0, ind]
			status = self.status_list[0, ind]
			xQ = self.xQ_list[:, ind]
			xL = self.xL_list[:, ind]
			vL = self.vL_list[:, ind]
			aL = self.aL_list[:, ind]
			daL = self.daL_list[:, ind]
			d2aL = self.d2aL_list[:, ind]
			d3aL = self.d3aL_list[:, ind]
			d4aL = self.d4aL_list[:, ind]
			return {'time': time, 'status': status, 'xQ': xQ, 'xL': xL, 'vL': vL, 'aL': aL, 'daL': daL, 'd2aL': d2aL, 'd3aL': d3aL, 'd4aL': d4aL}
		else:
			gamma = (time-self.time_list[0, ind])/(self.time_list[0, ind+1] - self.time_list[0, ind])
			# linear interpolation between poses
			time = self.time_list[0, ind]*gamma+self.time_list[0, ind+1]*(1-gamma) 
			status = self.status_list[0, ind]*gamma+self.status_list[0, ind+1]*(1-gamma)
			xQ = self.xQ_list[:, ind]*gamma+self.xQ_list[:, ind+1]*(1-gamma)
			xL = self.xL_list[:, ind]*gamma+self.xL_list[:, ind+1]*(1-gamma)
			vL = self.vL_list[:, ind]*gamma+self.vL_list[:, ind+1]*(1-gamma)
			aL = self.aL_list[:, ind]*gamma+self.aL_list[:, ind+1]*(1-gamma)
			daL = self.daL_list[:, ind]*gamma+self.daL_list[:, ind+1]*(1-gamma)
			d2aL = self.d2aL_list[:, ind]*gamma+self.d2aL_list[:, ind+1]*(1-gamma)
			d3aL = self.d3aL_list[:, ind]*gamma+self.d3aL_list[:, ind+1]*(1-gamma)
			d4aL = self.d4aL_list[:, ind]*gamma+self.d4aL_list[:, ind+1]*(1-gamma)
		
		return {'time': time, 'status': status, 'xQ': xQ, 'xL': xL, 'vL': vL, 'aL': aL, 'daL': daL, 'd2aL': d2aL, 'd3aL': d3aL, 'd4aL': d4aL}

	def interpolation_quadratic(self, time):
		pass

		
