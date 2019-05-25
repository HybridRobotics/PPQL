classdef GlobalSetting < handle
	properties
		has_obstaclelist;
		obstaclelist;
		has_closed_space;
		closed_space;
	end
	
	methods
		function obj = GlobalSetting()
			obj.has_obstaclelist = false;
			obj.obstaclelist = {};
			obj.has_closed_space = false;
		end
		
		function addObstacle(obj,obs)
			obj.has_obstaclelist = obj.has_obstaclelist | true;
			num_obstacle = size(obj.obstaclelist,2);
			obj.obstaclelist{num_obstacle+1} = obs;
		end
	end
end