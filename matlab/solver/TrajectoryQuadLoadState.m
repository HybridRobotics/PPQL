classdef TrajectoryQuadLoadState < handle
	properties
		traj = QuadLoadState.empty;
	end
	methods
		function addState(obj,state)
			obj.traj = [obj.traj,state];
		end
		
		function num = size(obj)
			num = size(obj.traj,2);
		end
		
		function [] = visualize(obj)
			for i = 1:obj.size()
				state = obj.traj(i);
				state.visualize();
				hold on;
			end
		end
	end
end