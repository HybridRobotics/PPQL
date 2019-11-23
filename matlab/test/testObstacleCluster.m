clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(4,8);

global_setting = GlobalSetting();
obstacle1 = Polyhedron('lb',[-0.25;-0.25;0.50],'ub',[0.25;0.25;2.00]);
obstacle2 = Polyhedron('lb',[-1.75;-0.25;0.50],'ub',[-1.25;0.25;2.00]);
obstacle3 = Polyhedron('lb',[1.25;-0.25;0.50],'ub',[1.75;0.25;2.00]);
obstacle4 = Polyhedron('lb',[-0.25;-1.75;0.50],'ub',[0.25;-1.25;2.00]);
obstacle5 = Polyhedron('lb',[-0.25;1.25;0.50],'ub',[0.25;1.75;2.00]);
obstacle6 = Polyhedron('lb',[-1.75;1.25;0.50],'ub',[-1.25;1.75;2.00]);
obstacle7 = Polyhedron('lb',[1.25;-1.75;0.50],'ub',[1.75;-1.25;2.00]);
global_setting.addObstacle(obstacle1);
global_setting.addObstacle(obstacle2);
global_setting.addObstacle(obstacle3);
global_setting.addObstacle(obstacle4);
global_setting.addObstacle(obstacle5);
global_setting.addObstacle(obstacle6);
global_setting.addObstacle(obstacle7);
region = Polyhedron('lb',[-2;-2;0.75],'ub',[2;2;1.25]);
global_setting.addClosedSpace(region);

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addGlobalSetting(global_setting);

start_state = QuadLoadState([-1.5;-1.5;1.0],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
end_state = QuadLoadState([1.5;1.5;1.0],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
IG = InitialGuess(params, path_planning_setting, start_state, end_state);
initial_guess = IG.solveInitialGuess();
problem = PathPlanningFormulation(params,path_planning_setting,start_state,end_state);
problem.setupInitialGuess(initial_guess);
compute_time = problem.solve(path_planning_setting);
fprintf('Computation Time in IPOPT: %f\n', compute_time);
problem.visualize(path_planning_setting);
traj = problem.getTrajectory();
traj.saveTrajReport('testObstacleNoGuidance.mat');