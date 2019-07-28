clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(1,4);
local_setting1.addWaypoint(Waypoint(25,'type','load','position',[0;0;2.5],'position_error',0.01));

global_setting = GlobalSetting();
obstacle1 = Polyhedron('lb',[-0.5;-0.5;1],'ub',[0.5;0.5;2]);
global_setting.addObstacle(obstacle1);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

initial = QuadLoadState([0;-2.3;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
final = QuadLoadState([0;1.7;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
problem = PathPlanningFormulation(params,path_planning_setting,initial,final);
problem.solve(path_planning_setting);
problem.visualize(path_planning_setting);
traj = problem.getTrajectory();
traj.saveTrajReport('testObstacleWithGuidance.mat');