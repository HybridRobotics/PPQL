classdef TrajectoryQuadLoadState < handle
	properties
		traj = QuadLoadDifferentialState.empty;
	end
	methods
		function addState(obj,state)
			obj.traj = [obj.traj, state];
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

		function [] = saveTrajReport(obj, filename)
			if ~isequal(exist('data','dir'),7)
				mkdir('data')
			end
			[time_list, status_list, xQ_list, xL_list, vL_list aL_list, daL_list, d2aL_list, d3aL_list, d4aL_list] = obj.getTrajReport();
			save(strcat('data/', filename),'time_list', 'status_list', 'xQ_list', 'xL_list','vL_list','aL_list','daL_list','d2aL_list','d3aL_list','d4aL_list');
		end

		function [time_list, status_list, xQ_list, xL_list, vL_list, aL_list, daL_list, d2aL_list, d3aL_list, d4aL_list] = getTrajReport(obj)
			% get the report (flat outputs and states) from the trajectory
			time_list = obj.getTimeList();
			status_list = obj.getStatusList();
			xQ_list = obj.getxQList();
			xL_list = obj.getxLList();
			vL_list = obj.getvLList();
			aL_list = obj.getaLList();
			daL_list = obj.getaLList();
			d2aL_list = obj.getaLList();
			d3aL_list = obj.getaLList();
			d4aL_list = obj.getaLList();
		end

		function time_list = getTimeList(obj)
			time_list = [];
			for i = 1:obj.size()
				time_list = [time_list, obj.traj(i).time];
			end
		end

		function status_list = getStatusList(obj)
			status_list = [];
			for i = 1:obj.size()
				status_list = [status_list, obj.traj(i).status];
			end
		end

		function xQ_list = getxQList(obj)
			% when cable is taut, we should include quadrotor position
			% no matter slack or taut, each payload state will always have position
			xQ_list = [];
			for i = 1:obj.size()
				xQ_list = [xQ_list, obj.traj(i).xQ];
			end
		end

		function xL_list = getxLList(obj)
			% no matter slack or taut, each payload state will always have position
			xL_list = [];
			for i = 1:obj.size()
				xL_list = [xL_list, obj.traj(i).xL];
			end
		end

		function vL_list = getvLList(obj)
			% no matter slack or taut, each payload state will always have velocity
			vL_list = [];
			for i = 1:obj.size()
				vL_list = [vL_list, obj.traj(i).vL];
			end
		end

		function aL_list = getaLList(obj)
			% no matter slack or taut, each payload state will always have acceleration
			aL_list = [];
			for i = 1:obj.size()
				aL_list = [aL_list, obj.traj(i).aL];
			end
		end

		function daL_list = getdaLList(obj)
			daL_list = [];
			for i = 1:obj.size()
				daL_list = [daL_list, obj.traj(i).daL];
			end
		end

		function d2aL_list = getd2aLList(obj)
			d2aL_list = [];
			for i = 1:obj.size()
				d2aL_list = [d2aL_list, obj.traj(i).d2aL];
			end
		end

		function d3aL_list = getd3aLList(obj)
			d3aL_list = [];
			for i = 1:obj.size()
				d3aL_list = [d3aL_list, obj.traj(i).d3aL];
			end
		end

		function d3aL_list = getd4aLList(obj)
			d4aL_list = [];
			for i = 1:obj.size()
				d4aL_list = [d4aL_list, obj.traj(i).d4aL];
			end
		end
	end
end