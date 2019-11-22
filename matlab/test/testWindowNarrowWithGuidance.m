clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.num_nodes = 10;
local_setting1.defineTraveltimeBounded(0,1);
local_setting1.status = 1;
% local_setting2.sample_distance_max = 0.05;

local_setting2 = LocalSetting();
local_setting2.num_nodes = 10;
local_setting2.defineTraveltimeBounded(0,1);
local_setting2.status = 2;
local_setting2.sample_distance_max = 0.02;

local_setting3 = LocalSetting();
local_setting3.num_nodes = 10;
local_setting3.defineTraveltimeBounded(0,1);
local_setting3.status = 1;
% local_setting2.sample_distance_max = 0.05;

global_setting = GlobalSetting();
obstacle1 = Polyhedron('lb',[-5; -0.05; -5], 'ub',[5; 0.05; 1.6]);
obstacle2 = Polyhedron('lb',[-5; -0.05; 2.3], 'ub', [5; 0.05; 9]);
obstacle3 = Polyhedron('lb',[-5; -0.05; -5], 'ub', [-1; 0.05; 9]);
obstacle4 = Polyhedron('lb',[1; -0.05; -5], 'ub', [5; 0.05; 9]);
global_setting.addObstacle(obstacle1);
global_setting.addObstacle(obstacle2);
global_setting.addObstacle(obstacle3);
global_setting.addObstacle(obstacle4);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addLocalSetting(local_setting2);
path_planning_setting.addLocalSetting(local_setting3);
path_planning_setting.addGlobalSetting(global_setting);

start_state = QuadLoadState([0;-0.5;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
end_state = QuadLoadState([0;0.5;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,start_state,end_state);
problem.solve(path_planning_setting);
problem.visualize(path_planning_setting);
traj = problem.getTrajectory();
traj.saveTrajReport('testWindowNarrowWithGuidance.mat');