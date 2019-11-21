classdef GlobalSetting < handle
	properties
		has_obstaclelist;
		obstaclelist;
		has_closedspace;
		closedspacelist;
	end
	
	methods
		function obj = GlobalSetting()
			obj.has_obstaclelist = false;
			obj.obstaclelist = {};
			obj.has_closedspace = false;
			obj.closedspacelist = {};
		end
		
		function addObstacle(obj,obs)
			obj.has_obstaclelist = obj.has_obstaclelist | true;
			num_obstacle = size(obj.obstaclelist, 2);
			obj.obstaclelist{num_obstacle+1} = obs;
		end

		function addClosedSpace(obj, space)
			obj.has_closedspace = obj.has_obstaclelist | true;
			num_closedspace = size(obj.closedspacelist, 2);
			obj.closedspacelist{num_closedspace+1} = space;
		end
	end
end