clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(1,6);
local_setting1.addWaypoint(Waypoint(25,'load',[0;0;1.30],0.01));

global_setting = GlobalSetting();
obstacle1 = Polyhedron('lb',[-5; -0.25; -2], 'ub',[5; 0.25; 1.1]);
obstacle2 = Polyhedron('lb',[-5; -0.25; 1.9], 'ub', [5; 0.25; 4]);
obstacle3 = Polyhedron('lb',[-5; -0.25; -2], 'ub', [-1; 0.25; 4]);
obstacle4 = Polyhedron('lb',[1; -0.25; -2], 'ub', [5; 0.25; 4]);
global_setting.addObstacle(obstacle1);
global_setting.addObstacle(obstacle2);
global_setting.addObstacle(obstacle3);
global_setting.addObstacle(obstacle4);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

initial = QuadLoadState([0;-2.3;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
final = QuadLoadState([0;1.7;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,initial,final);
problem.solve(path_planning_setting);
problem.saveData('hover_to_hover_small_window.mat');
problem.visualize(path_planning_setting);