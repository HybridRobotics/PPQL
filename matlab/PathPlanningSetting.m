classdef PathPlanningSetting < handle
	properties
		has_global_setting;
		global_setting;
		has_local_setting;
		num_local_setting;
		local_setting_list; % list of local settings
	end
	
	methods
		function obj = PathPlanningSetting()
			obj.has_global_setting = false;
			obj.has_local_setting = false;
			obj.num_local_setting = 0;
		end
		
		function addGlobalSetting(obj,global_setting)
			obj.has_global_setting = true;
			obj.global_setting = global_setting;
		end
		
		function addLocalSetting(obj,local_setting)
			obj.has_local_setting = true;
			obj.num_local_setting = obj.num_local_setting+1;
			obj.local_setting_list{obj.num_local_setting} = local_setting;
		end
	end
end