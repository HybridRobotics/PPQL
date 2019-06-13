

classdef LocalSetting < handle
	properties
		% ========== Obstacle Avoidance ========== %
		dmin;
		epsilonx;
		epsilony;
		epsilonz;
		
		% ========== Optimization ========== %
		Q;
		Sf;
		R;
		Rbar;
		
		% ========== Physical Limitations ========== %
		cable_length_min; % minimal revolute length
		
		% ========== Navigation Specification ========== %
		is_fixed_traveltime;
		traveltime_min;
		traveltime_max;
		traveltime;
		num_nodes;
		sample_distance_max;
		
		% ========== Special Specification ========== %
		has_closed_space;
		closed_space;
		has_waypoints;
		waypoints;
		has_guidance_policy;
		guidance_policy;
		
		% ========== System Status ========== %
		status; % 1 (taut) 2(slack) 3(release)
	end
	
	methods
		function obj = LocalSetting()
			% ========== Obstacle Avoidance ========== %
			obj.dmin = 0.05;
			obj.epsilonx = 0.16;
			obj.epsilony = 0.16;
			obj.epsilonz = 0.05;
			% ========== Optimization ========== %
			obj.Q = diag([1;1;1]);
			obj.Sf = 10^8;
			obj.R = 10^6;
			obj.Rbar = 10^8;
			% ========== Physical Limitations ========== %
			obj.cable_length_min = 0.15;
			% ========== Navigation Specification ========== %
			obj.num_nodes = 50;
			% ========== Special Specification ========== %
			obj.has_waypoints = false;
			obj.waypoints = {};
			obj.status = 1; % 'taut', 'slack', 'release'
		end
		
		function defineTraveltimeFixed(obj,traveltime)
			obj.is_fixed_traveltime = true;
			obj.traveltime = traveltime;
		end
		
		function defineTraveltimeBounded(obj,lowerbound,upperbound)
			obj.is_fixed_traveltime = false;
			obj.traveltime_min = lowerbound;
			obj.traveltime_max = upperbound;
			obj.traveltime = sdpvar(1);
		end
		
		function addWaypoint(obj,waypoint)
			obj.has_waypoints = obj.has_waypoints | true;
			num_waypoints = size(obj.waypoints,2);
			obj.waypoints{num_waypoints+1} = waypoint;
			if waypoint.ind_node > obj.num_nodes + 1
				msg = 'Error occured during the waypoint specification. The specified node of waypoint should be less or equal to the number of nodes in each segment';
				error(msg);
			end
		end
	end
end