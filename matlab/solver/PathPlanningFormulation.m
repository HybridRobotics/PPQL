classdef PathPlanningFormulation < handle
	properties
		params;
		start_state;
		end_state;

		vars;
		constr;

		cost_time;
		cost_derivative;
		cost_tension;
		cost_cable;
		cost;

		options;

		output;
	end
	
	methods
		function obj = PathPlanningFormulation(params,pps,start_state,end_state)
			% constructor
			
			% system parameters
			obj.params = params;
			% boundary conditions
			obj.start_state = start_state;
			obj.end_state = end_state;
			% initialization of constraints
			obj.constr = [];
			% initilization of cost
			obj.cost_time = 0;
			obj.cost_derivative = 0;
			obj.cost_tension = 0;
			obj.cost_cable = 0;
			obj.cost = 0;
			% initialization of variables
			obj.initVariables(pps);
			% setup optimization
			obj.setupOptimization(pps);
			% ipopt options
			obj.setupOptions();
		end
		
		function initVariables(obj,pps)
			% Variables for local settings
			obj.vars = cell(1,pps.num_local_setting);
			if pps.has_local_setting
				for i = 1:pps.num_local_setting
					ls = pps.local_setting_list{i};
					if ls.status == 1
						obj.vars{i}.xL = sdpvar(3,ls.num_nodes+1);
						assign(obj.vars{i}.xL, [linspace(0,0,ls.num_nodes+1);linspace(-1.5,1.5,ls.num_nodes+1);linspace(1.0,1.0,ls.num_nodes+1)]);
						obj.vars{i}.vL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.aL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.daL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.d2aL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.d3aL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.d4aL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.q = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.F = sdpvar(1,ls.num_nodes+1);
						obj.vars{i}.L = sdpvar(1,ls.num_nodes+1);
						obj.vars{i}.traveltime = ls.traveltime;
						obj.vars{i}.timestep = ls.traveltime/ls.num_nodes;
						obj.vars{i}.num_nodes = ls.num_nodes;
					else
						obj.vars{i}.xL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.vL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.aL = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.q = sdpvar(3,ls.num_nodes+1);
						obj.vars{i}.F = sdpvar(1,ls.num_nodes+1);
						obj.vars{i}.L = sdpvar(1,ls.num_nodes+1);
						obj.vars{i}.traveltime = ls.traveltime;
						obj.vars{i}.timestep = ls.traveltime/ls.num_nodes;
						obj.vars{i}.num_nodes = ls.num_nodes;
					end
				end
			else
				msg = 'Error occured during the initialization of variables. The optimization problem should have at least one local setting.';
				error(msg);
			end
			
			% Variables for global settings
			if pps.has_global_setting
				for i = 1:pps.num_local_setting
					ls = pps.local_setting_list{i};
					% obstacle avoidance
					for j = 1:size(pps.global_setting.obstaclelist,2)
						obstacle = pps.global_setting.obstaclelist{j};
						obj.vars{i}.lambda{j} = sdpvar(size(obstacle.A,1),ls.num_nodes+1);
						obj.vars{i}.mu{j} = sdpvar(size(obstacle.b,1),ls.num_nodes+1);
					end
				end
			end
		end
		
		function setupOptimization(obj,pps)
			fprintf('Adding constraints\n');
			obj.addConstraint(pps);
			fprintf('Adding cost functions\n');
			obj.addCost(pps);
		end
		
		function addConstraint(obj,pps)
			obj.addConstraintStartState(pps);
			obj.addConstraintEndState(pps);
			obj.addConstraintCollocation(pps);
			obj.addConstraintAcc(pps);
			% obj.addConstraintJerk(pps);
			obj.addConstraintTime(pps);
			obj.addConstraintComplementary(pps);
			obj.addConstraintGeometric(pps);
			obj.addConstraintSlackVariable(pps);
			obj.addConstraintObstacleAvoidance(pps);
			obj.addConstraintWaypoints(pps);
			obj.addConstraintMaximumTravelDistance(pps);
			obj.addConstraintClosedSpace(pps);
		end
		
		function addConstraintStartState(obj,pps)
			% Set variables according to start_state
			if obj.start_state.status == 1
				obj.constr = obj.constr + [obj.vars{1}.xL(:,1) == obj.start_state.xL;...
											 obj.vars{1}.vL(:,1) == obj.start_state.vL;...
											 obj.vars{1}.aL(:,1) == [0;0;0];...
											 obj.vars{1}.daL(:,1) == [0;0;0];...
											 obj.vars{1}.d2aL(:,1) == [0;0;0];...
											 obj.vars{1}.d3aL(:,1) == [0;0;0];...
											 obj.vars{1}.d4aL(:,1) == [0;0;0];...
											 ];
			else
				obj.constr = obj.constr + [obj.vars{1}.xL(:,1) == obj.start_state.xL;...
											 obj.vars{1}.vL(:,1) == obj.start_state.vL;...
											 ];
			end
		end
		
		function addConstraintEndState(obj,pps)
			% Set variables according to end_state
			end_state_constraint_type = 0;
			K = 10^8;
			if obj.end_state.status == 1
				if end_state_constraint_type == 0
					% hard constraints
					obj.constr = obj.constr + [obj.vars{end}.xL(:,end) == obj.end_state.xL;...
												 obj.vars{end}.vL(:,end) == obj.end_state.vL;...
												 obj.vars{end}.aL(:,end) == [0;0;0];...
												 obj.vars{end}.daL(:,end) == [0;0;0];...
												 obj.vars{end}.d2aL(:,end) == [0;0;0];...
												 obj.vars{end}.d3aL(:,end) == [0;0;0];...
												 obj.vars{end}.d4aL(:,end) == [0;0;0];...
												 ];
				elseif end_state_constraint_type == 1
					% optimization-based
					obj.constr = obj.constr + [obj.vars{end}.xL(:,end) == obj.end_state.xL];
					e_vL = obj.vars{end}.vL(:,end) - obj.end_state.vL;
					e_aL = obj.vars{end}.aL(:,end) - [0;0;0];
					e_daL = obj.vars{end}.daL(:,end) - [0;0;0];
					e_d2aL = obj.vars{end}.d2aL(:,end) - [0;0;0];
					e_d3aL = obj.vars{end}.d3aL(:,end) - [0;0;0];
					e_d4aL = obj.vars{end}.d4aL(:,end) - [0;0;0];
					obj.cost = obj.cost + K*(e_vL'*e_vL + e_aL'*e_aL + e_daL'*e_daL + e_d2aL'*e_d2aL + e_d3aL'*e_d3aL + e_d4aL'*e_d4aL);
				end
			else
				obj.constr = obj.constr + [obj.vars{end}.xL(:,end) == obj.end_state.xL;...
											 obj.vars{end}.aL(:,end) == [0;0;-9.8];
											 ];
			end
		end
		
		function addConstraintCollocation(obj,pps)
			collocation_constraint_type = 1;
			for i = 1:pps.num_local_setting
				if pps.local_setting_list{i}.status == 1
					fprintf('cable is taut\n');
					for j = 2:obj.vars{i}.num_nodes+1
						if collocation_constraint_type == 0
							% collocation constraint: zero order
							obj.constr = obj.constr + ...
								[obj.vars{i}.xL(:,j)-obj.vars{i}.xL(:,j-1) == 0.5*(obj.vars{i}.vL(:,j)+obj.vars{i}.vL(:,j-1))*obj.vars{i}.timestep;...
								 obj.vars{i}.vL(:,j)-obj.vars{i}.vL(:,j-1) == 0.5*(obj.vars{i}.aL(:,j)+obj.vars{i}.aL(:,j-1))*obj.vars{i}.timestep;...
								 obj.vars{i}.aL(:,j)-obj.vars{i}.aL(:,j-1) == 0.5*(obj.vars{i}.daL(:,j)+obj.vars{i}.daL(:,j-1))*obj.vars{i}.timestep;...
								 obj.vars{i}.daL(:,j)-obj.vars{i}.daL(:,j-1) == 0.5*(obj.vars{i}.d2aL(:,j)+obj.vars{i}.d2aL(:,j-1))*obj.vars{i}.timestep;...
								 obj.vars{i}.d2aL(:,j)-obj.vars{i}.d2aL(:,j-1) == 0.5*(obj.vars{i}.d3aL(:,j)+obj.vars{i}.d3aL(:,j-1))*obj.vars{i}.timestep;...
								 obj.vars{i}.d3aL(:,j)-obj.vars{i}.d3aL(:,j-1) == 0.5*(obj.vars{i}.d4aL(:,j)+obj.vars{i}.d4aL(:,j-1))*obj.vars{i}.timestep;...
								 ];
                        elseif collocation_constraint_type == 1
							 % collocation constraint: first order
							 obj.constr = obj.constr + ...
								[obj.vars{i}.xL(:,j)-obj.vars{i}.xL(:,j-1) == obj.vars{i}.vL(:,j-1)*obj.vars{i}.timestep;...
								 obj.vars{i}.vL(:,j)-obj.vars{i}.vL(:,j-1) == obj.vars{i}.aL(:,j-1)*obj.vars{i}.timestep;...
								 obj.vars{i}.aL(:,j)-obj.vars{i}.aL(:,j-1) == obj.vars{i}.daL(:,j-1)*obj.vars{i}.timestep;...
								 obj.vars{i}.daL(:,j)-obj.vars{i}.daL(:,j-1) == obj.vars{i}.d2aL(:,j-1)*obj.vars{i}.timestep;...
								 obj.vars{i}.d2aL(:,j)-obj.vars{i}.d2aL(:,j-1) == obj.vars{i}.d3aL(:,j-1)*obj.vars{i}.timestep;...
								 obj.vars{i}.d3aL(:,j)-obj.vars{i}.d3aL(:,j-1) == obj.vars{i}.d4aL(:,j-1)*obj.vars{i}.timestep;...
								 ];
						end
					end
				else
					fprintf('cable is slack: payload free falling or throwing\n');
					% free fall or payload throwing
					for j = 2:obj.vars{i}.num_nodes+1
						obj.constr = obj.constr + ...
						[obj.vars{i}.aL(:,j-1) == [0;0;-9.8];...
						obj.vars{i}.vL(:,j) == obj.vars{i}.vL(:,j-1) + obj.vars{i}.aL(:,j-1)*obj.vars{i}.timestep;...
						obj.vars{i}.xL(:,j) == obj.vars{i}.xL(:,j-1) + obj.vars{i}.vL(:,j-1)*obj.vars{i}.timestep + 0.5 * obj.vars{i}.aL(:,j-1) * obj.vars{i}.timestep^2;...
						];
						% high order derivatives are all zero during free fall
						
					end
				end
			end
			% continuity of trajectory
			for i = 2:pps.num_local_setting
				obj.constr = obj.constr + ...
					[obj.vars{i}.xL(:,1) == obj.vars{i-1}.xL(:,end);...
					obj.vars{i}.vL(:,1) == obj.vars{i-1}.vL(:,end);...
					obj.vars{i}.q(:,1) == obj.vars{i-1}.q(:,end);...
					obj.vars{i}.L(1) == obj.vars{i-1}.L(end);...
					];
				if pps.local_setting_list{i-1}.status == 1 && pps.local_setting_list{i}.status == 1
					obj.constr = obj.constr + ...
					[obj.vars{i}.aL(:,1) == obj.vars{i-1}.aL(:,end);...
					obj.vars{i}.daL(:,1) == obj.vars{i-1}.daL(:,end);...
					obj.vars{i}.d2aL(:,1) == obj.vars{i-1}.d2aL(:,end);...
					obj.vars{i}.d3aL(:,1) == obj.vars{i-1}.d3aL(:,end);...
					obj.vars{i}.d4aL(:,1) == obj.vars{i-1}.d4aL(:,end);
					];
				end
			end
		end

		function addConstraintAcc(obj,pps)
			for i = 1:pps.num_local_setting
				if pps.local_setting_list{i}.status ~= 1
					continue;
				end
				if ~isempty(pps.local_setting_list{i}.acc_min)
					for j = 2:obj.vars{i}.num_nodes + 1
						obj.constr = obj.constr + [pps.local_setting_list{i}.acc_min <= obj.vars{i}.aL(:,j)];
					end
				end

				if ~isempty(pps.local_setting_list{i}.acc_max)
					for j = 2:obj.vars{i}.num_nodes + 1
						obj.constr = obj.constr + [obj.vars{i}.aL(:,j) <= pps.local_setting_list{i}.acc_max];
					end
				end
			end
		end
		
		function addConstraintJerk(obj,pps)
			for i = 1:pps.num_local_setting
				if pps.local_setting_list{i}.status ~= 1
					continue;
				end
				if ~isempty(pps.local_setting_list{i}.jerk_min)
					for j = 2:obj.vars{i}.num_nodes + 1
						obj.constr = obj.constr + [pps.local_setting_list{i}.jerk_min <= obj.vars{i}.daL(:,j)];
					end
				end

				if ~isempty(pps.local_setting_list{i}.jerk_max)
					for j = 2:obj.vars{i}.num_nodes + 1
						obj.constr = obj.constr + [obj.vars{i}.daL(:,j) <= pps.local_setting_list{i}.jerk_max];
					end
                end
			end
		end
		
		function addConstraintTime(obj,pps)
			for i = 1:pps.num_local_setting
				if ~pps.local_setting_list{i}.is_fixed_traveltime
				obj.constr = obj.constr + [pps.local_setting_list{i}.traveltime_min <= obj.vars{i}.traveltime <= pps.local_setting_list{i}.traveltime_max];
				end
			end
		end
		
		function addConstraintComplementary(obj,pps)
			for i = 1:pps.num_local_setting
				for j = 1:obj.vars{i}.num_nodes+1
					obj.constr = obj.constr + [-obj.vars{i}.F(j)*obj.vars{i}.q(:,j) == obj.params.mL*(obj.params.g*[0; 0; 1] + obj.vars{i}.aL(:,j));...
												 obj.vars{i}.F(j) >= 0;...
												 obj.vars{i}.F(j)*(obj.params.L0 - obj.vars{i}.L(j)) == 0];
				end
			end
		end
		
		function addConstraintGeometric(obj,pps)
			for i = 1:pps.num_local_setting
				for j = 1:obj.vars{i}.num_nodes+1
					obj.constr = obj.constr + [obj.vars{i}.q(:,j)'*obj.vars{i}.q(:,j) == 1;... % S^2 condition
												 obj.vars{i}.q(:,j).*[0;0;-1] >= 0;... % payload should be under the quadrotor
												 pps.local_setting_list{i}.cable_length_min <= obj.vars{i}.L(j) <= obj.params.L0]; % cable length limitation
				end
			end
		end
		
		function addConstraintSlackVariable(obj,pps)
			if pps.has_global_setting
				for i = 1:pps.num_local_setting
					% positivity of slack variables
					for j = 1:size(pps.global_setting.obstaclelist,2)
						obj.constr = obj.constr + [obj.vars{i}.lambda{j} >= 0];
						obj.constr = obj.constr + [obj.vars{i}.mu{j} >= 0];
					end
				end
			end
		end
		
		function addConstraintObstacleAvoidance(obj,pps)
			obstacle_avoidance_type = 0; % 0 for minimum distance, 1 for minimum penetration
			if pps.global_setting.has_obstaclelist
				AA = [-eye(3);eye(3)]; % time-variant
				for i = 1:pps.num_local_setting
					for j = 1:obj.vars{i}.num_nodes+1
						% direct calculation
						q = obj.vars{i}.q(:,j);
						qq = q + [0;0;-1];
						qq_norm = sdpvar(1,1);
						obj.constr = obj.constr + [qq_norm^2 == q(1)^2 + q(2)^2 + (q(3)-1)^2];
						% Rodrigues' Formula
						R = eye(3) + hat_map(qq)*sin(qq_norm*pi)/qq_norm + hat_map(qq)*hat_map(qq)*(1-cos(qq_norm*pi))/(qq_norm^2);
						bb = [pps.local_setting_list{i}.epsilonx;...
							pps.local_setting_list{i}.epsilony;...
							0;... % assume payload is point-mass
							pps.local_setting_list{i}.epsilonx;...
							pps.local_setting_list{i}.epsilony;...
							pps.local_setting_list{i}.epsilonz + obj.vars{i}.L(j)];
						for k = 1:size(pps.global_setting.obstaclelist,2)
							A = pps.global_setting.obstaclelist{k}.A;
							b = pps.global_setting.obstaclelist{k}.b;
							if obstacle_avoidance_type == 0
								obj.constr = obj.constr+...
								[-bb'*obj.vars{i}.mu{k}(:,j)+(A*obj.vars{i}.xL(:,j)-b)'*obj.vars{i}.lambda{k}(:,j) >= pps.local_setting_list{i}.dmin;...
								AA'*obj.vars{i}.mu{k}(:,j)+R'*A'*obj.vars{i}.lambda{k}(:,j) == 0;...
								obj.vars{i}.lambda{k}(:,j)'*A*A'*obj.vars{i}.lambda{k}(:,j) <= 1];
							elseif obstacle_avoidance_type == 1
								dist = sdpvar(1);
								obj.constr = obj.constr+...
								[-bb'*obj.vars{i}.mu{k}(:,j)+(A*obj.vars{i}.xL(:,j)-b)'*obj.vars{i}.lambda{k}(:,j) >= pps.local_setting_list{i}.dmin-dist;...
								dist >= 0;...
								AA'*obj.vars{i}.mu{k}(:,j)+R'*A'*obj.vars{i}.lambda{k}(:,j) == 0;...
								obj.vars{i}.lambda{k}(:,j)'*A*A'*obj.vars{i}.lambda{k}(:,j) == 1];
							obj.cost = obj.cost + dist * pps.local_setting_list{i}.K;
							end
						end
					end
				end
			end
		end
		
		function addConstraintWaypoints(obj,pps)
			for i = 1:pps.num_local_setting
				waypoints = pps.local_setting_list{i}.waypoints;
				num_waypoints = size(waypoints,2);
				for j = 1:num_waypoints
					% position constraint
					if ~isempty(waypoints{j}.position)
						if strcmp(waypoints{j}.type,'load')
							obj.constr = obj.constr + [-waypoints{j}.position_error<=...
								obj.vars{i}.xL(:,waypoints{j}.ind_node) - waypoints{j}.position<=...
								waypoints{j}.position_error];
						elseif strcmp(waypoints{j}.type,'quad')
							obj.constr = obj.constr + [-waypoints{j}.position_error<=...
								obj.vars{i}.xL(:,waypoints{j}.ind_node)-...
								obj.vars{i}.L(waypoints{j}.ind_node)*obj.vars{i}.q(:, waypoints{j}.ind_node)-...
								waypoints{j}.position<=...
								waypoints{j}.position_error];
						else
							msg = 'Error occured during the waypoints navigation constraints: inappropriate waypoint type';
							error(msg);
						end
					end
					
					% attitude constraint (vectorial form)
					if ~isempty(waypoints{j}.attitude)
						obj.constr = obj.constr + [-waypoints{j}.attitude_error<=...
							obj.vars{i}.q(:,waypoints{j}.ind_node) - waypoints{j}.attitude<=...
							waypoints{j}.attitude_error];
					end
					
					% attitude constraint (scalar form)
					if ~isempty(waypoints{j}.attitude_angle)
						obj.constr = obj.constr + [cos(waypoints{j}.attitude_angle + waypoints{j}.attitude_angle_error)<=...
							vec_dot(obj.vars{i}.q(:,waypoints{j}.ind_node), [0;0;-1])<=...
							cos(waypoints{j}.attitude_angle - waypoints{j}.attitude_angle_error)];
					end
					
					% feasible region constraint
					if ~isempty(waypoints{j}.feasible_region)
						obj.constr = obj.constr + [waypoints{j}.feasible_region.A * obj.vars{i}.xL(:,waypoints{j}.ind_node)<=waypoints{j}.feasible_region.b];
					end
				end
			end
		end
		
		function addConstraintMaximumTravelDistance(obj,pps)
			for i = 1:pps.num_local_setting
				if ~isempty(pps.local_setting_list{i}.sample_distance_max)
					max_dis = pps.local_setting_list{i}.sample_distance_max;
					if i ~= 1
						obj.constr = obj.constr + [(obj.vars{i}.xL(:,1) - obj.vars{i}.xL(:,end))'*(obj.vars{i}.xL(:,1) - obj.vars{i}.xL(:,end)) <= max_dis^2];
					end
					for j = 2:pps.local_setting_list{i}.num_nodes+1
						obj.constr = obj.constr + [(obj.vars{i}.xL(:,j) - obj.vars{i}.xL(:,j-1))'*(obj.vars{i}.xL(:,j) - obj.vars{i}.xL(:,j-1)) <= max_dis^2];
					end
				end
			end
		end

		function addConstraintClosedSpace(obj,pps)
			% Add locally closed space constraints
			for i = 1:pps.num_local_setting
				if ~isempty(pps.local_setting_list{i}.closedspacelist)
					num_closed_space = size(pps.local_setting_list{i}.closedspacelist, 2);
					for j = 1: num_closed_space
						for k = 2: obj.vars{i}.num_nodes+1
							obj.constr = obj.constr + [pps.local_setting_list{i}.closedspacelist{j}.A * obj.vars{i}.xL(:,k) <= pps.local_setting_list{i}.closedspacelist{j}.b];
						end
					end
				end
			end
			% Add globally closed space constraints
			if pps.global_setting.has_closedspace
				num_closed_space = size(pps.global_setting.closedspacelist, 2);
				for i = 1: num_closed_space
					for j = 1:pps.num_local_setting
						for k = 2:obj.vars{j}.num_nodes+1
							obj.constr = obj.constr + [pps.global_setting.closedspacelist{i}.A * obj.vars{j}.xL(:,k) <= pps.global_setting.closedspacelist{i}.b];
						end
					end
				end
			end
		end
		
		function addCost(obj,pps)
			obj.addCostTime(pps);
			obj.addCostCollocation(pps);
			obj.cost = obj.cost + obj.cost_time + obj.cost_derivative + obj.cost_tension + obj.cost_cable;
		end
		
		function addCostTime(obj,pps)
			for i = 1:pps.num_local_setting
				obj.cost_time = obj.cost_time + pps.local_setting_list{i}.Sf*pps.local_setting_list{i}.traveltime;
			end
		end
		
		function addCostCollocation(obj,pps)
			for i = 1:pps.num_local_setting
				if pps.local_setting_list{i}.status == 1
					for j = 2:obj.vars{i}.num_nodes+1
						obj.cost_derivative = obj.cost_derivative + (obj.vars{i}.d4aL(:,j)-obj.vars{i}.d4aL(:,j-1))'*...
							pps.local_setting_list{i}.Q*(obj.vars{i}.d4aL(:,j)-obj.vars{i}.d4aL(:,j-1));
						obj.cost_tension = obj.cost_tension + pps.local_setting_list{i}.R*obj.vars{i}.F(j);
						obj.cost_cable = obj.cost_cable + pps.local_setting_list{i}.Rbar*(obj.params.L0-obj.vars{i}.L(j));
					end
				end
			end
		end
		
		function setupInitialGuess(obj, ig)
			assign(obj.vars{1}.xL, ig.vars{1}.xL)
		end

		function setupOptions(obj)
			obj.options = sdpsettings('solver','ipopt','verbose',1,'usex0',1);
			% Termination
			obj.options.ipopt.tol = 1;
			obj.options.ipopt.dual_inf_tol = 1;
			obj.options.ipopt.constr_viol_tol = 10^(-2);
			obj.options.ipopt.compl_inf_tol = 10^(-2);
			obj.options.ipopt.max_iter = 3000;
		end
		
		function [compute_time] = solve(obj,pps)
			fprintf('Optimization begins to be solved with IPOPT Solver\n')
			res = optimize(obj.constr,obj.cost,obj.options);
			fprintf('Optimization ends\n')
			obj.mergeTrajectory(pps);
			compute_time = res.solvertime;
		end
		
		function mergeTrajectory(obj,pps)
			% merge multiple trajectories together
			traj = TrajectoryQuadLoadState();
			time = 0; % TODO: should be extracted from start_state state
			init_state = QuadLoadDifferentialState();
			init_state.flatOutputsToStateTaut(time,...
				double(obj.vars{1}.xL(:,1)),...
				double(obj.vars{1}.vL(:,1)),...
				double(obj.vars{1}.aL(:,1)),...
				double(obj.vars{1}.daL(:,1)),...
				double(obj.vars{1}.d2aL(:,1)),...
				double(obj.vars{1}.d3aL(:,1)),...
				double(obj.vars{1}.d4aL(:,1)),...
				double(obj.vars{1}.q(:,1)),...
				double(obj.vars{1}.L(:,1)));
			traj.addState(init_state);
			for i = 1:pps.num_local_setting
				if pps.local_setting_list{i}.status == 1
					for j = 2:obj.vars{i}.num_nodes+1
						state = QuadLoadDifferentialState();
						time = time + double(obj.vars{i}.timestep);
						state.flatOutputsToStateConnect(time,...
							double(obj.vars{i}.xL(:,j)),...
							double(obj.vars{i}.vL(:,j)),...
							double(obj.vars{i}.aL(:,j)),...
							double(obj.vars{i}.daL(:,j)),...
							double(obj.vars{i}.d2aL(:,j)),...
							double(obj.vars{i}.d3aL(:,j)),...
							double(obj.vars{i}.d4aL(:,j)),...
							double(obj.vars{i}.q(:,j)),...
							double(obj.vars{i}.L(:,j)));
						traj.addState(state);
					end
				elseif pps.local_setting_list{i}.status == 2
					for j = 2:obj.vars{i}.num_nodes+1
						state = QuadLoadDifferentialState();
						time = time + double(obj.vars{i}.timestep);
						state.flatOutputsToStateSlack(time,...
							double(obj.vars{i}.xL(:,j)),...
							double(obj.vars{i}.vL(:,j)),...
							double(obj.vars{i}.aL(:,j)),...
							double(obj.vars{i}.q(:,j)),...
							double(obj.vars{i}.L(:,j)));
						traj.addState(state);
					end
				else
					for j = 2:obj.vars{i}.num_nodes+1
						state = QuadLoadDifferentialState();
						time = time + double(obj.vars{i}.timestep);
						state.flatOutputsToStateRelease(time,...
							double(obj.vars{i}.xL(:,j)),...
							double(obj.vars{i}.vL(:,j)),...
							double(obj.vars{i}.aL(:,j)),...
							double(obj.vars{i}.q(:,j)),...
							double(obj.vars{i}.L(:,j)));
						state.status = 3;
						traj.addState(state);
					end
				end
			end
			obj.output.trajectory = traj;
		end
		
		function visualize(obj,pps)
			figure;
			obj.visualizeTrajectory(pps);
			hold on;
			obj.visualizeSettings(pps);
			hold on;
		end
		
		function visualizeSettings(obj,pps)
			% display obstacles
			obj.visualizeObstacles(pps);
		end
		
		function visualizeObstacles(obj,pps)
			for i = 1:size(pps.global_setting.obstaclelist,2)
				pps.global_setting.obstaclelist{i}.plot();
				hold on;
			end
		end
		
		function visualizeTrajectory(obj,pps)
			obj.output.trajectory.visualize();
			hold on;
		end
		
		function traj = getTrajectory(obj)
			traj = obj.output.trajectory;
		end
	end
end