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
				finish = false;
				% Break if goal node is already reached
				for j = 1:1:length(nodes)
					if norm(nodes(j).coord - q_goal.coord) < 0.25
						finish = true;
						break;
					end
				end
				if finish == true
					fprintf('reach the target position');
					break;
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
% 			obj.visualizeObstacles()
			start = obj.start_state.xL';
			goal = obj.end_state.xL';
			dx = 0.1; dy = 0.1; dz = 0.1;
			temp1 = obj.x_bound(1):dx:obj.x_bound(2);
			temp2 = obj.y_bound(1):dy:obj.y_bound(2);
			temp3 = obj.z_bound(1):dz:obj.z_bound(2);
			MAP = zeros((obj.x_bound(2)-obj.x_bound(1))/dx+1,...
				(obj.y_bound(2)-obj.y_bound(1))/dy+1,...
				(obj.z_bound(2)-obj.z_bound(1))/dz+1,3);
			weight = 1;
			% assign position into nodes
			for x = 1:size(MAP,1)
				for y = 1:size(MAP,2)
					for z = 1:size(MAP,3)
						MAP(x,y,z,1) = temp1(x);
						MAP(x,y,z,2) = temp2(y);
						MAP(x,y,z,3) = temp3(z);
					end
				end
			end
			% heuristic Map of all nodes
			for x = 1:size(MAP,1)
				for y = 1:size(MAP,2)
					for z = 1:size(MAP,3)
						point = [MAP(x,y,z,1);MAP(x,y,z,2);MAP(x,y,z,3)];
						if(obj.isObstacleFree(point))
							H(x,y,z) = weight*norm(goal-point);
							G(x,y,z) = inf;
						else
							feasible(x,y,z) = inf;
						end
					end
				end
			end
			% initial condition
			start_index_x = (start(1)-obj.x_bound(1))/dx + 1;
			start_index_y = (start(2)-obj.y_bound(1))/dy + 1;
			start_index_z = (start(3)-obj.z_bound(1))/dz + 1;
			G(start_index_x,start_index_y,start_index_z) = 0;
			F(start_index_x,start_index_y,start_index_z) = H(start_index_x,start_index_y,start_index_z);
			closedNodes = [];
			openNodes = [start_index_x start_index_y start_index_z G(start_index_x,start_index_y,start_index_z) F(start_index_x,start_index_y,start_index_z) 0];
			solved = false;
			while (~isempty(openNodes))
				% find node from open set with smallest F value
				[A, I] = min(openNodes(:,5));
				% set current node
				current = openNodes(I, :);
 				% plot3(current(1),current(2),current(3),'o','color','g','MarkerFaceColor','g')
				% if goal is reached, break the loop
				if([MAP(current(1),current(2),current(3),1),MAP(current(1),current(2),current(3),2),MAP(current(1),current(2),current(3),3)]==goal)
					closedNodes = [closedNodes;current];
					solved = true;
					break;
				end
				%remove current node from open set and add it to closed set
				openNodes(I,:) = [];
				closedNodes = [closedNodes;current];
				%for all neighbors of current node
				for x = current(1)-1:current(1)+1
					for y = current(2)-1:current(2)+1
						for z = current(3)-1:current(3)+1
							% if out of range skip
							if (x<1||x>size(MAP,1)||y<1||y>size(MAP,2)||z<1||z>size(MAP,3))
								continue
							end
							% if object skip
							if (isinf(feasible(x,y,z)))
								continue
							end
							% if current node skip
							if (x==current(1) && y==current(2) && z==current(3))
								continue
							end
							% if already in closed set skip
							skip = 0;
							for j = 1:size(closedNodes,1)
								if(x == closedNodes(j,1) && y==closedNodes(j,2) && z==closedNodes(j,3))
									skip = 1;
									break;
								end
							end
							if(skip == 1)
								continue
							end
							A = [];
							% Check if already in open set
							if(~isempty(openNodes))
								for j = 1:size(openNodes,1)
									if(x == openNodes(j,1) && y==openNodes(j,2) && z==openNodes(j,3))
										A = j;
										break;
									end
								end
							end
							newG = G(current(1),current(2),current(3)) + round(norm([(current(1)-x)*dx,(current(2)-y)*dy,(current(3)-z)*dz]));
							% if not in open set, add to open set
							if(isempty(A))
								G(x,y,z) = newG;
								newF = G(x,y,z) + H(x,y,z);
								newNode = [x y z G(x,y,z) newF size(closedNodes,1)];
								openNodes = [openNodes; newNode];
								plot3(x,y,z,'x','color','b')
								continue
							end
							%if no better path, skip
							if (newG >= G(x,y,z))
								continue
							end
							G(x,y,z) = newG;
							newF = newG + H(x,y,z);
							openNodes(A,4:6) = [newG newF size(closedNodes,1)];
						end
					end
				end
			end
			if (solved)
				% Path plotting
				j = size(closedNodes,1);
				path = [];
				while(j > 0)
					point = [MAP(closedNodes(j,1),closedNodes(j,2),closedNodes(j,3),1),MAP(closedNodes(j,1),closedNodes(j,2),closedNodes(j,3),2),MAP(closedNodes(j,1),closedNodes(j,2),closedNodes(j,3),3)];
					j = closedNodes(j,6);
					path = [point;path];
				end
				plot3(path(:,1),path(:,2),path(:,3),'-','color','r')
			else
				disp('No Path Found')
			end
			feasible_traj = path';
			toc
		end
		
		function [flag] = isObstacleFree(obj, point)
			pps_copy = obj.pps;
			if pps_copy.global_setting.has_obstaclelist
				for k = 1:size(pps_copy.global_setting.obstaclelist,2)
					% buffer region
					obstacle = pps_copy.global_setting.obstaclelist{k};
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