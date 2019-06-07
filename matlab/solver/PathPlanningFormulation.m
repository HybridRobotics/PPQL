classdef PathPlanningFormulation < handle
	properties
		params;
		initial;
		final;
		
		vars;
		constr;
		cost;
		options;
		
		output;
	end
	
	methods
		function obj = PathPlanningFormulation(params,pps,initial,final)
			% constructor
			
			% system parameters
			obj.params = params;
			% boundary conditions
			obj.initial = initial;
			obj.final = final;
			% initialization of constraints
			obj.constr = [];
			% initilization of cost
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
					obj.vars{i}.xL = sdpvar(3,ls.num_nodes+1);
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
			obj.addConstraintInitial(pps);
			obj.addConstraintFinal(pps);
			obj.addConstraintTime(pps);
			obj.addConstraintCollocation(pps);
			obj.addConstraintComplementary(pps);
			obj.addConstraintGeometric(pps);
			obj.addConstraintSlackVariable(pps);
			obj.addConstraintObstacleAvoidance(pps);
			obj.addConstraintWaypoints(pps);
		end
		
		function addConstraintInitial(obj,pps)
			% Set variables according to initial condition
			if obj.initial.status
				obj.constr = obj.constr + [obj.vars{1}.xL(:,1) == obj.initial.xL;...
											 obj.vars{1}.vL(:,1) == obj.initial.vL;...
											 obj.vars{1}.aL(:,1) == [0;0;0];...
											 obj.vars{1}.daL(1:2,1) == [0;0];...
											 obj.vars{1}.d2aL(1:2,1) == [0;0];...
											 obj.vars{1}.d3aL(1:2,1) == [0;0];...
											 ];
			else
				obj.constr = obj.constr + [obj.vars{1}.xL(:,1) == obj.initial.xL;...
											 obj.vars{1}.vL(:,1) == obj.initial.vL;...
											 ];
			end
		end
		
		function addConstraintFinal(obj,pps)
			% Set variables according to final condition
			if obj.final.status
				obj.constr = obj.constr + [obj.vars{end}.xL(:,end) == obj.final.xL;...
											 obj.vars{end}.vL(:,end) == obj.final.vL;...
											 obj.vars{end}.aL(:,end) == [0;0;0];...
											 obj.vars{end}.daL(:,end) == [0;0;0];...
											 obj.vars{end}.d2aL(:,end) == [0;0;0];...
											 obj.vars{end}.d3aL(:,end) == [0;0;0];...
											 obj.vars{end}.d4aL(:,end) == [0;0;0];...
											 ];
			else
				obj.constr = obj.constr + [obj.vars{end}.xL(:,end) == obj.final.xL;...
											 obj.vars{end}.vL(:,end) == obj.final.vL;...
											 ];
			end
		end
		
		function addConstraintCollocation(obj,pps)
			for i = 1:pps.num_local_setting
				for j = 2:obj.vars{i}.num_nodes+1
					obj.constr = obj.constr + ...
						[obj.vars{i}.xL(:,j)-obj.vars{i}.xL(:,j-1) == 0.5*(obj.vars{i}.vL(:,j)+obj.vars{i}.vL(:,j-1))*obj.vars{i}.timestep;...
						 obj.vars{i}.vL(:,j)-obj.vars{i}.vL(:,j-1) == 0.5*(obj.vars{i}.aL(:,j)+obj.vars{i}.aL(:,j-1))*obj.vars{i}.timestep;...
						 obj.vars{i}.aL(:,j)-obj.vars{i}.aL(:,j-1) == 0.5*(obj.vars{i}.daL(:,j)+obj.vars{i}.daL(:,j-1))*obj.vars{i}.timestep;...
						 obj.vars{i}.daL(:,j)-obj.vars{i}.daL(:,j-1) == 0.5*(obj.vars{i}.d2aL(:,j)+obj.vars{i}.d2aL(:,j-1))*obj.vars{i}.timestep;...
						 obj.vars{i}.d2aL(:,j)-obj.vars{i}.d2aL(:,j-1) == 0.5*(obj.vars{i}.d3aL(:,j)+obj.vars{i}.d3aL(:,j-1))*obj.vars{i}.timestep;...
						 obj.vars{i}.d3aL(:,j)-obj.vars{i}.d3aL(:,j-1) == 0.5*(obj.vars{i}.d4aL(:,j)+obj.vars{i}.d4aL(:,j-1))*obj.vars{i}.timestep;...		 
						 ];
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
												 obj.vars{i}.F(j)*(obj.params.L - obj.vars{i}.L(j)) == 0];
				end
			end
		end
		
		function addConstraintGeometric(obj,pps)
			for i = 1:pps.num_local_setting
				for j = 1:obj.vars{i}.num_nodes+1
					obj.constr = obj.constr + [obj.vars{i}.q(:,j)'*obj.vars{i}.q(:,j) == 1;... % S^2 condition
												 obj.vars{i}.q(:,j).*[0;0;-1] >= 0;... % payload should be under the quadrotor
												 pps.local_setting_list{i}.cable_length_min <= obj.vars{i}.L(j) <= obj.params.L]; % cable length limitation
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
			if pps.global_setting.has_obstaclelist
				AA = [-eye(3);eye(3)]; % time-variant
				for i = 1:pps.num_local_setting
					for j = 1:obj.vars{i}.num_nodes+1
						% Rodrigues' formula
						qq = (1/2)*([0;0;-1]+obj.vars{i}.q(:,j));
						% R = eye(3) + sin(pi)*hat(qq) +...
						% 	(1-cos(pi))*hat(qq)*hat(qq);
						R = eye(3) + 2*hat(qq)*hat(qq);
						bb = [pps.local_setting_list{i}.epsilonx;...
							  pps.local_setting_list{i}.epsilony;...
							  pps.local_setting_list{i}.epsilonz;...
							  pps.local_setting_list{i}.epsilonx;...
							  pps.local_setting_list{i}.epsilony;...
							  pps.local_setting_list{i}.epsilonz + obj.vars{i}.L(j)];
						for k = 1:size(pps.global_setting.obstaclelist,2)
							A = pps.global_setting.obstaclelist{k}.A;
							b = pps.global_setting.obstaclelist{k}.b;
							obj.constr = obj.constr + ...
							[-bb'*obj.vars{i}.mu{k}(:,j)+(A*obj.vars{i}.xL(:,j)-b)'*obj.vars{i}.lambda{k}(:,j) >= pps.local_setting_list{i}.dmin;...
							AA'*obj.vars{i}.mu{k}(:,j)+R'*A'*obj.vars{i}.lambda{k}(:,j) == 0;...
							obj.vars{i}.lambda{k}(:,j)'*A*A'*obj.vars{i}.lambda{k}(:,j) <= 1];
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
					if strcmp(waypoints{j}.type_waypoint,'load')
						obj.constr = obj.constr + [-waypoints{j}.position_error<= obj.vars{i}.xL(:,waypoints{j}.ind_node) - waypoints{j}.position <= waypoints{j}.position_error];
					elseif strcmp(waypoints{j}.type_waypoint,'quad')
						obj.constr = obj.constr + [-waypoints{j}.position_error<= obj.vars{i}.xL(:,waypoints{j}.ind_node) - obj.vars{i}.L(waypoints{j}.ind_node) * obj.vars{i}.q(:, waypoints{j}.ind_node) - waypoints{j}.position <= waypoints{j}.position_error];
					else
						msg = 'Error occured during the waypoints navigation constraints';
						error(msg);
					end
				end
			end
		end
		
		function addCost(obj,pps)
			obj.addCostTime(pps);
			obj.addCostCollocation(pps);
		end
		
		function addCostTime(obj,pps)
			for i = 1:pps.num_local_setting
				obj.cost = obj.cost + pps.local_setting_list{i}.Sf*pps.local_setting_list{i}.traveltime; 
			end
		end
		
		function addCostCollocation(obj,pps)
			for i = 1:pps.num_local_setting
				for j = 1:obj.vars{i}.num_nodes+1
					obj.cost = obj.cost+obj.vars{i}.d4aL(:,j)'*pps.local_setting_list{i}.Q*obj.vars{i}.d4aL(:,j)+...
						pps.local_setting_list{i}.R*obj.vars{i}.F(j)+pps.local_setting_list{i}.Rbar*(obj.params.L-obj.vars{i}.L(j));
				end
			end
		end
		
		function setupOptions(obj)
			obj.options = sdpsettings('solver','ipopt','verbose',1);
			% Termination
			obj.options.ipopt.tol = 10^(-2);
			obj.options.ipopt.dual_inf_tol = 1;
			obj.options.ipopt.constr_viol_tol = 10^(-3);
			obj.options.ipopt.compl_inf_tol = 10^(-3);
			obj.options.ipopt.max_iter = 1000;
			% NLP scaling
			obj.options.ipopt.nlp_scaling_method = 'gradient-based';
			% Barrier parameters
			obj.options.ipopt.mu_strategy = 'adaptive';
		end
		
		function solve(obj,pps)
			fprintf('Optimization begins to be solved with IPOPT Solver\n')
			optimize(obj.constr,obj.cost,obj.options);
			fprintf('Optimization ends\n')
			obj.mergeTrajectory(pps);
		end
		
		function mergeTrajectory(obj,pps)
			% merge multiple trajectories together
			t_list = [0];
			xl_list = double(obj.vars{1}.xL(:,1));
			vl_list = double(obj.vars{1}.vL(:,1));
			al_list = double(obj.vars{1}.aL(:,1));
			dal_list = double(obj.vars{1}.daL(:,1));
			d2al_list = double(obj.vars{1}.d2aL(:,1));
			d3al_list = double(obj.vars{1}.d3aL(:,1));
			d4al_list = double(obj.vars{1}.d4aL(:,1));
			time_accum = 0;
			
			for i = 1:pps.num_local_setting
				time_segment = linspace(time_accum, double(obj.vars{i}.traveltime), obj.vars{i}.num_nodes+1);
				t_list = [t_list, time_segment(2:end)];
				time_accum = time_accum + time_segment;
				xl_list = [xl_list, double(obj.vars{1}.xL(:,2:obj.vars{i}.num_nodes+1))];
				vl_list = [vl_list, double(obj.vars{1}.vL(:,2:obj.vars{i}.num_nodes+1))];
				al_list = [al_list, double(obj.vars{1}.aL(:,2:obj.vars{i}.num_nodes+1))];
				dal_list = [dal_list, double(obj.vars{1}.daL(:,2:obj.vars{i}.num_nodes+1))];
				d2al_list = [d2al_list, double(obj.vars{1}.d2aL(:,2:obj.vars{i}.num_nodes+1))];
				d3al_list = [d3al_list, double(obj.vars{1}.d3aL(:,2:obj.vars{i}.num_nodes+1))];
				d4al_list = [d4al_list, double(obj.vars{1}.d4aL(:,2:obj.vars{i}.num_nodes+1))];
			end
			obj.output.trajectory = Trajectory(t_list,xl_list,vl_list,al_list,dal_list,d2al_list,d3al_list,d4al_list);
		end
		
		function visualize(obj,pps)
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
		
		function saveData(obj,filename)
			if ~isequal(exist('data','dir'),7)                          
				mkdir('data')
			end
			traj = obj.output.trajectory;
			time = traj.t_list;
			xl = traj.xL_list;
			vl = traj.vL_list;
			al = traj.aL_list;
			dal = traj.daL_list;
			d2al = traj.d2aL_list;
			d3al = traj.d3aL_list;
			d4al = traj.d4aL_list;
			save(strcat('data/', filename),'time','xl','vl','al','dal','d2al','d3al','d4al');
		end
	end
end