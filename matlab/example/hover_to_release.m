clear
close all

params = SystemParameters();

local_setting1 = LocalSetting();
local_setting1.defineTraveltimeBounded(1,4);
local_setting1.status = 1; % default
local_setting2 = LocalSetting();
local_setting2.num_nodes = 10;
local_setting2.defineTraveltimeBounded(0.3,1);
local_setting2.status = 3;

global_setting = GlobalSetting();

path_planning_setting = PathPlanningSetting();
path_planning_setting.addLocalSetting(local_setting1);
path_planning_setting.addLocalSetting(local_setting2);
path_planning_setting.addGlobalSetting(global_setting);

initial = QuadLoadState([0;-2.3;1.5],[0;0;0],[0;0;-1],[0;0;0],eye(3),[0;0;0]);
initial.status = 1;
final = QuadLoadState([0;1.7;1.5],[0;0;0]);
final.status = 3;
problem = PathPlanningFormulation(params,path_planning_setting,initial,final);
problem.solve(path_planning_setting);
problem.saveData('hover_to_release.mat');
problem.visualize(path_planning_setting);
