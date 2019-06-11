classdef Waypoint < handle
	properties
		ind_node;
		
		type; % load or quadrotor tracking
		position;
		position_error;
		
		attitude;
		attitude_error;
		
		attitude_angle;
		attitude_angle_error;
		
		feasible_region;
	end
	methods
		function obj = Waypoint(varargin)
			if mod(nargin,2) ~= 1
				msg = 'Error occurred during the waypoint construction';
				error(msg);
			else
				obj.ind_node = varargin{1};
				len = floor(nargin/2);
				for i = 1:len
					if strcmp(varargin{2*i},'type')
						obj.addType(varargin{2*i+1});
					elseif strcmp(varargin{2*i},'position')
						obj.addPosition(varargin{2*i+1});
					elseif strcmp(varargin{2*i},'position_error')
						obj.addPositionError(varargin{2*i+1});
					elseif strcmp(varargin{2*i},'attitude')
						obj.addAttitude(varargin{2*i+1});
					elseif strcmp(varargin{2*i},'attitude_error')
						obj.addAttitudeError(varargin{2*i+1});
					elseif strcmp(varargin{2*i},'attitude_angle')
						obj.addAttitudeAngle(varargin{2*i+1});
					elseif strcmp(varargin{2*i},'attitude_angle_error')
						obj.addAttitudeAngleError(varargin{2*i+1});
					elseif strcmp(varargin{2*i},'feasible_region')
						obj.addFeasibleRegion(varargin{2*i+1});
					else
						msg = 'Error occurred during the waypoint construction: undefined specification';
						error(msg);
					end
				end
			end
		end
		
		% waypoint position
		function addType(obj,type)
			obj.type = type;
		end
		
		function addPosition(obj,position)
			obj.position = position;
		end
		
		function addPositionError(obj,position_error)
			obj.position_error = position_error;
		end
		
		% waypoint attitude (vectorial form)
		function addAttitude(obj,attitude)
			obj.attitude = attitude;
		end
		
		function addAttitudeError(obj,attitude_error)
			obj.attitude_error = attitude_error;
		end
		
		% waypoint attitude (scalar form)
		function addAttitudeAngle(obj,attitude_angle)
			obj.attitude_angle = attitude_angle;
		end
		
		function addAttitudeAngleError(obj,attitude_angle_error)
			obj.attitude_angle_error = attitude_angle_error;
		end
		
		function addFeasibleRegion(obj,feasible_region)
			obj.feasible_region = feasible_region;
		end
	end
end