clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(4,8);
local_setting1.sample_distance_max = 0.25;

global_setting = GlobalSetting();
obstacle1 = Polyhedron('lb',[-5; -0.5; -3], 'ub',[5; 0.5; 1.4]);
obstacle2 = Polyhedron('lb',[-5; -0.5; 2.6], 'ub', [5; 0.5; 7]);
obstacle3 = Polyhedron('lb',[-5; -0.5; -3], 'ub', [-1; 0.5; 7]);
obstacle4 = Polyhedron('lb',[1; -0.5; -3], 'ub', [5; 0.5; 7]);
global_setting.addObstacle(obstacle1);
global_setting.addObstacle(obstacle2);
global_setting.addObstacle(obstacle3);
global_setting.addObstacle(obstacle4);
% region = Polyhedron('lb',[-5; -2; -3], 'ub', [5; 2; 7]);
% global_setting.addClosedSpace(region);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

start_state = QuadLoadState([0;-1.5;1.0],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
end_state = QuadLoadState([0;1.5;1.0],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,start_state,end_state);
compute_time = problem.solve(path_planning_setting);
fprintf('Computation Time in IPOPT: %f\n', compute_time);
problem.visualize(path_planning_setting);
traj = problem.getTrajectory();
traj.saveTrajReport('testWindowBig.mat');