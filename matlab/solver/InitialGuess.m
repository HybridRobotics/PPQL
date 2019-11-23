classdef InitialGuess < handle
	properties
		params;
		pps;
		start_state;
		end_state;
		x_bound = [-2, 2];
		y_bound = [-2, 2];
		z_bound = [-2, 2];
	end
	methods
		function obj = InitialGuess(params, pps, start_state, end_state)
			obj.params = params;
			obj.pps = pps;
			obj.start_state = start_state;
			obj.end_state = end_state;
		end
		
		function ig = solveInitialGuess(obj)
			% assume only one segment
			feasible_traj = obj.astar();
			ls = obj.pps.local_setting_list{1};
			if ls.status == 1
				if ~obj.pps.local_setting_list{1}.is_fixed_traveltime
					ig.vars{1}.timestep = 0.5*(obj.pps.local_setting_list{1}.traveltime_min + obj.pps.local_setting_list{1}.traveltime_max); 
				end
				timesteps_feasible = linspace(0, ig.vars{1}.timestep, size(feasible_traj,2));
				timesteps = linspace(0, ig.vars{1}.timestep, obj.pps.local_setting_list{1}.num_nodes+1);
				ig.vars{1}.xL(1,:) = interp1(timesteps_feasible, feasible_traj(1,:), timesteps);
				ig.vars{1}.xL(2,:) = interp1(timesteps_feasible, feasible_traj(2,:), timesteps);
				ig.vars{1}.xL(3,:) = interp1(timesteps_feasible, feasible_traj(3,:), timesteps);
			else
				
			end
		end
		
		function feasible_traj = rrt(obj)
			tic
			
			EPS = 0.5;
			numNodes = 500;        

			q_start.coord = obj.start_state.xL;
			q_start.cost = 0;
			q_start.parent = 0;
			q_goal.coord = obj.end_state.xL;
			q_goal.cost = 0;

			nodes(1) = q_start;
			figure(1)
			obj.visualizeObstacles()
			xlim(obj.x_bound);
			ylim(obj.y_bound);
			zlim(obj.z_bound);
			
			for i = 1:1:numNodes
				q_rand = obj.getRandomPoint();
				plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410])

				% Break if goal node is already reached
				for j = 1:1:length(nodes)
					if norm(nodes(j).coord - q_goal.coord) < 0.25
						break
					end
				end

				% Pick the closest node from existing list to branch out from
				ndist = [];
				for j = 1:1:length(nodes)
					n = nodes(j);
					tmp = norm(n.coord - q_rand);
					ndist = [ndist tmp];
				end
				[val, idx] = min(ndist);
				q_near = nodes(idx);

				q_new.coord = q_near.coord + (q_rand - q_near.coord)/val * EPS;
				line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
				drawnow
				hold on
				q_new.cost = norm(q_new.coord - q_near.coord) + q_near.cost;

				% Within a radius of r, find all existing nodes
				q_nearest = [];
				r = 0.5;
				neighbor_count = 1;
				for j = 1:1:length(nodes)
					if (norm(nodes(j).coord - q_new.coord)) <= r
						q_nearest(neighbor_count).coord = nodes(j).coord;
						q_nearest(neighbor_count).cost = nodes(j).cost;
						neighbor_count = neighbor_count+1;
					end
				end

				% Initialize cost to currently known value
				q_min = q_near;
				C_min = q_new.cost;

				% Iterate through all nearest neighbors to find alternate lower
				% cost paths

				for k = 1:1:length(q_nearest)
					if q_nearest(k).cost + norm(q_nearest(k).coord - q_new.coord) < C_min
						q_min = q_nearest(k);
						C_min = q_nearest(k).cost + norm(q_nearest(k).coord - q_new.coord);
						line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');            
						hold on
					end
				end

				% Update parent to least cost-from node
				for j = 1:1:length(nodes)
					if nodes(j).coord == q_min.coord
						q_new.parent = j;
					end
				end

				% Append to nodes
				nodes = [nodes q_new];
			end

			D = [];
			for j = 1:1:length(nodes)
				tmpdist = norm(nodes(j).coord - q_goal.coord);
				D = [D tmpdist];
			end

			% Search backwards from goal to start to find the optimal least cost path
			[val, idx] = min(D);
			q_final = nodes(idx);
			q_goal.parent = idx;
			q_end = q_goal;
			nodes = [nodes q_goal];
			feasible_traj = [q_end.coord];
			while q_end.parent ~= 0
				start = q_end.parent;
				line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
				hold on
				q_end = nodes(start);
				feasible_traj = [feasible_traj, q_end.coord];
			end
			toc
			fprintf('RRT planning finished');
		end
		
		function feasible_traj = astar(obj)
			tic
			figure(1)
			xlim(obj.x_bound);
			ylim(obj.y_bound);
			zlim(obj.z_bound);
			obj.visualizeObstacles()
			dx = 0.2;
			move_up = [0;0;dx]; move_down = [0;0;-dx];
			move_toward = [dx;0;0]; move_back = [-dx;0;0];
			move_left = [0;-dx;0]; move_right = [0;dx;0];
			actions = [move_up,move_down,move_toward,move_back,move_left,move_right];
			feasible_traj = [obj.start_state.xL];
			pos = obj.start_state.xL;
			while norm(pos - obj.end_state.xL) >= sqrt(3)*0.1
				cost_list = [];
				valid_actions = [];
				for i = 1:6
					if obj.isObstacleFree(pos + actions(:,i))
						valid_actions = [valid_actions actions(:,i)]; 
						cost_list = [cost_list norm(actions(:,i))+norm(obj.end_state.xL-pos-actions(:,i))];
					end
				end
				[val, idx] = min(cost_list);
				pos_new = pos + valid_actions(:,idx);
				line([pos(1), pos_new(1)], [pos(2), pos_new(2)], [pos(3), pos_new(3)], 'Color', 'r', 'LineWidth', 4);
				hold on
				pos = pos_new;
				feasible_traj = [feasible_traj pos];
			end
			toc
		end
		
		function [flag] = isObstacleFree(obj, point)
			pps_copy = obj.pps;
			if pps_copy.global_setting.has_obstaclelist
				for k = 1:size(pps_copy.global_setting.obstaclelist,2)
					% buffer region
					obstacle = pps_copy.global_setting.obstaclelist{k};
					obstacle.b
					b_new = [obstacle.b(1:2)+0.25;obstacle.b(3)+1;obstacle.b(4:5)+0.25;obstacle.b(6)];
					A_new = obstacle.A;
					if A_new * point <= b_new
						flag = false;
						return;
					end
				end
			end
			flag = true;
		end
		
		function point = getRandomPoint(obj)
			% initialize a point
			point = [obj.x_bound(1)+rand(1)*(obj.x_bound(2)-obj.x_bound(1));...
				obj.y_bound(1)+rand(1)*(obj.y_bound(2)-obj.y_bound(1));...
				obj.z_bound(1)+rand(1)*(obj.z_bound(2)-obj.z_bound(1))];
			while ~obj.isObstacleFree(point)
				point = [obj.x_bound(1)+rand(1)*(obj.x_bound(2)-obj.x_bound(1));...
					obj.y_bound(1)+rand(1)*(obj.y_bound(2)-obj.y_bound(1));...
					obj.z_bound(1)+rand(1)*(obj.z_bound(2)-obj.z_bound(1))];
			end
		end
		
		function visualizeSettings(obj)
			obj.visualizeObstacles();
		end
		
		function visualizeObstacles(obj)
			pps_copy = obj.pps;
			for i = 1:size(obj.pps.global_setting.obstaclelist,2)
				pps_copy.global_setting.obstaclelist{i}.plot();
				hold on;
			end
		end
	end
end