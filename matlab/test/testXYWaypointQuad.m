clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(1,6);
local_setting1.num_nodes = 100;
local_setting1.addWaypoint(Waypoint(26,'type','quad',...
	'position',[-1;2;1.5],...
	'position_error',0.01));
local_setting1.addWaypoint(Waypoint(51,'type','quad',...
	'position',[1;2;1.5],...
	'position_error',0.01));
local_setting1.addWaypoint(Waypoint(76,'type','quad',...
	'position',[1;0;1.5],...
	'position_error',0.01));

global_setting = GlobalSetting();

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

start_state = QuadLoadState([-1;0;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
end_state = QuadLoadState([-1;0;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,start_state,end_state);
problem.solve(path_planning_setting);
problem.visualize(path_planning_setting);
traj = problem.getTrajectory();
traj.saveTrajReport('testXYWaypointQuad.mat');