clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(1,4);
local_setting1.num_nodes = 100;
local_setting1.addWaypoint(Waypoint(26,'quad',[-1;2;1.5],0.01));
local_setting1.addWaypoint(Waypoint(51,'quad',[1;2;1.5],0.01));
local_setting1.addWaypoint(Waypoint(76,'quad',[1;0;1.5],0.01));

global_setting = GlobalSetting();

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

initial = QuadLoadState([-1;0;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
final = QuadLoadState([-1;0;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,initial,final);
problem.solve();
problem.saveData('hover_to_hover_three_waypoints_quad.mat');