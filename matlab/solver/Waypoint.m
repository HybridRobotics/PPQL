classdef Waypoint < handle
	properties
		ind_node;
		type_waypoint; % load or quadrotor tracking
		position;
		position_error;
	end
	methods
		function obj = Waypoint(ind_node, type_waypoint,position,position_error)
			obj.ind_node = ind_node;
			obj.type_waypoint = type_waypoint;
			obj.position = position;
			obj.position_error = position_error;
		end
	end
end