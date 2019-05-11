import scipy.io as sio
from scipy import interpolate

class TrajectoryCollector:
	def __init__(self,filename):
		data = sio.loadmat('../data/'+filename)
		self.time = data['time']
		self.xl = data['xl']
		self.vl = data['vl']
		self.al = data['al']
		self.dal = data['dal']
		self.d2al = data['d2al']
		self.d3al = data['d3al']
		self.d4al = data['d4al']
		self.num_nodes = self.time.shape[1]-1
		self.dt = (self.time[0,self.num_nodes]-self.time[0,0])/self.num_nodes

	def interpolation_linear(self, time):
		ind = int(time/self.dt)
		gamma = (time-self.dt*ind)/self.dt
		print()
		if ind < self.num_nodes:
			xl = self.xl[:,ind]*gamma + self.xl[:,ind+1]*(1-gamma)
			vl = self.vl[:,ind]*gamma + self.vl[:,ind+1]*(1-gamma)
			al = self.al[:,ind]*gamma + self.al[:,ind+1]*(1-gamma)
			dal = self.dal[:,ind]*gamma + self.dal[:,ind+1]*(1-gamma)
			d2al = self.d2al[:,ind]*gamma + self.d2al[:,ind+1]*(1-gamma)
			d3al = self.d3al[:,ind]*gamma + self.d3al[:,ind+1]*(1-gamma)
			d4al = self.d4al[:,ind]*gamma + self.d4al[:,ind+1]*(1-gamma)
		else:
			xl = self.xl[:,ind]
			vl = self.vl[:,ind]
			al = self.al[:,ind]
			dal = self.dal[:,ind]
			d2al = self.d2al[:,ind]
			d3al = self.d3al[:,ind]
			d4al = self.d4al[:,ind]
		
		return {'xl': xl, 'vl': vl, 'al': al, 'dal': dal, 'd2al': d2al, 'd3al': d3al, 'd4al': d4al}

	def interpolation_quadratic(self, time):
		pass

		
