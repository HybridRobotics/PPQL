clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.num_nodes = 20;
local_setting1.defineTraveltimeBounded(1,3);
local_setting1.status = 1;
local_setting2 = LocalSetting();
local_setting2.num_nodes = 10;
local_setting2.defineTraveltimeBounded(0.3,1);
local_setting2.status = 2;
local_setting3 = LocalSetting();
local_setting3.num_nodes = 20;
local_setting3.defineTraveltimeBounded(1,3);
local_setting3.status = 1;
local_setting1.sample_distance_max = 0.1;

global_setting = GlobalSetting();
obstacle1 = Polyhedron('lb',[-5; -0.05; -5], 'ub',[5; 0.05; 1.5]);
obstacle2 = Polyhedron('lb',[-5; -0.05; 2.1], 'ub', [5; 0.05; 10]);
obstacle3 = Polyhedron('lb',[-5; -0.05; -5], 'ub', [-1; 0.05; 10]);
obstacle4 = Polyhedron('lb',[1; -0.05; -5], 'ub', [5; 0.05; 10]);
global_setting.addObstacle(obstacle1);
global_setting.addObstacle(obstacle2);
global_setting.addObstacle(obstacle3);
global_setting.addObstacle(obstacle4);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addLocalSetting(local_setting2);
path_planning_setting.addLocalSetting(local_setting3);
path_planning_setting.addGlobalSetting(global_setting);

initial = QuadLoadState([0;-1;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
final = QuadLoadState([0;1;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,initial,final);
problem.solve(path_planning_setting);
problem.visualize(path_planning_setting);
traj = problem.getTrajectory();
traj.saveTrajReport('hover_to_hover_small_window_slack.mat');