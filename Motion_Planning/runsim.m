% if demoActivate exist and equal true, not clean all variable.
% runsim.m called by the demo,  not clean all variable
% runsim.m called by itself,  clean all variable
if (~exist('demoActivate')) || (demoActivate == false)
    close all;
    clear all;
    clc;
end

addpath(genpath('./'));


%% map generation
map = GridMap();
init_data
map.flag_obstacles();


%% path planning
nquad = length(start);
for qn = 1:nquad    
    disp('JPS time is :');
    tic
    path{2*qn-1} = JPS_3D(map, start{qn}, stop{qn});
    path{2*qn-1} = [start{qn}; path{2*qn-1}(1:end,:)];
    path{2*qn-1}(end + 1,:) = stop{qn};
    toc
end

%% delete the points of no-use
for qn = 1:nquad
    path{2*qn} = simplify_path(map, path{2*qn-1});
end

%% Generating Convex Polytopes
obps = PointCloudMap(map.blocks, map.margin);   % blocks of Metric Map change to point cloud

disp('JPS -> SFC time is :');
tic
for qn = 1:nquad
    decomps{2*qn-1} = SFC_3D(path{2*qn}, obps, map.boundary);%  call SFC
    decomps{2*qn} = {};
end 
toc

%% draw path and blocks
% if nquad == 1
%     plot_path(path, map, decomps); 
% else
% %     decomps{1,2}
% %     decomps{3:4}
%     % you could modify your plot_path to handle cell input for multiple robots
% %     for qn = 1:nquad
% %         plot_path({path{1:2}}, map, {decomps{1},decomps{1}}); 
% %         plot_path({path{3:4}}, map, {decomps{3},decomps{4}});   
% %     end
%     plot_path(path, map, decomps)
% end

% draw_ObcPoints
% makeGifAndJpg(1);     %figure(1): Graph without trajectory

%% Trajectory planning
% [t_time, ts_par, x_par] = TrajectoryPlanning(path{path_id}, decomps{SFC_id}, time_allocation);
% [t_time1, ts_par1, x_par1] = TrajectoryPlanning(path{4}, decomps{3}, time_allocation);

%% Trajectory tracking
disp('Generating Trajectory ...');
trajectory = test_trajectory(start, stop, path, true, map, decomps, time_allocation);
disp('Blue line is Trajectory planning.');
disp('Red line is Trajectory tracking.');

%% Gif
% makeGifAndJpg(3);     %figure(3): Graph with trajectory
