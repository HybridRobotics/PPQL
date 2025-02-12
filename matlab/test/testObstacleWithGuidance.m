clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(2,6);
local_setting1.addWaypoint(Waypoint(25,'type','load','position',[0;0;2.5],'position_error',0.01));

global_setting = GlobalSetting();
obstacle1 = Polyhedron('lb',[-0.5;-0.5;1],'ub',[0.5;0.5;1.5]);
global_setting.addObstacle(obstacle1);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

start_state = QuadLoadState([0;-1.5;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
end_state = QuadLoadState([0;1.5;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,start_state,end_state);
compute_time = problem.solve(path_planning_setting);
fprintf('Computation Time in IPOPT: %f\n', compute_time);
problem.visualize(path_planning_setting);
traj = problem.getTrajectory();
traj.saveTrajReport('testObstacleWithGuidance.mat');