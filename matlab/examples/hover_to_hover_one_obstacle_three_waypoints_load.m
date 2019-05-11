clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(1,6);
local_setting1.num_nodes = 100;
local_setting1.addWaypoint(Waypoint(26,'load',[0;0;0.5],0.01));
local_setting1.addWaypoint(Waypoint(51,'load',[1;0;1.5],0.01));
local_setting1.addWaypoint(Waypoint(76,'load',[0;0;2.5],0.01));

global_setting = GlobalSetting();
obstacle1 = Polyhedron('lb',[-0.5;0.5;-.5],'ub',[0.5;0.5;0.5]);
global_setting.addObstacle(obstacle1);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

initial = QuadLoadState([-1;0;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
final = QuadLoadState([-1;0;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,initial,final);
problem.solve();
problem.saveData('results.mat');