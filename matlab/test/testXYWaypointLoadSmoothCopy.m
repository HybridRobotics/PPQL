clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(4,8);
local_setting1.num_nodes = 100;
local_setting1.R = 10^6;
local_setting1.addWaypoint(Waypoint(26,'type','load',...
	'position',[0.0;2.0;1.5],...
	'position_error',0.01));
local_setting1.addWaypoint(Waypoint(51,'type','load',...
	'position',[2.0;2.0;1.5],...
	'position_error',0.01));
local_setting1.addWaypoint(Waypoint(76,'type','load',...
	'position',[2.0;0.0;1.5],...
	'position_error',0.01));

global_setting = GlobalSetting();
region = Polyhedron('lb',[-1;-1;1.0],'ub',[4;4;2.0]);
global_setting.addClosedSpace(region);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

start_state = QuadLoadState([0;0;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
end_state = QuadLoadState([0;0;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,start_state,end_state);
compute_time = problem.solve(path_planning_setting);
fprintf('Computation Time in IPOPT: %f\n', compute_time);
problem.visualize(path_planning_setting);
xlim([-0.5 4.25]);
ylim([-0.5 4.25]);
axis equal;
view(2)
traj = problem.getTrajectory();
traj.saveTrajReport('testXYWaypointLoad.mat');