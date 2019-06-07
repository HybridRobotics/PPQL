classdef Trajectory
	properties
		num_nodes;
		t_list;
		xL_list;
		vL_list;
		aL_list;
		daL_list;
		d2aL_list;
		d3aL_list;
		d4aL_list;
	end
	methods
		function obj = Trajectory(t_list, xL_list, vL_list, aL_list, daL_list, d2aL_list, d3aL_list, d4aL_list)
			% get flat outputs
			obj.t_list = t_list;
			obj.xL_list = xL_list;
			obj.vL_list = vL_list;
			obj.aL_list = aL_list;
			obj.daL_list = daL_list;
			obj.d2aL_list = d2aL_list;
			obj.d3aL_list = d3aL_list;
			obj.d4aL_list = d4aL_list;
		end
		
		function [] = visualize(obj)
			for i = 1:length(obj.t_list)
				state = QuadLoadState();
				state.flatOutputsToStates(obj.xL_list(:,i), obj.vL_list(:,i), obj.aL_list(:,i), obj.daL_list(:,i), obj.d2aL_list(:,i), obj.d3aL_list(:,i), obj.d4aL_list(:,i));
				state.visualize();
				hold on;
			end
		end
	end
end