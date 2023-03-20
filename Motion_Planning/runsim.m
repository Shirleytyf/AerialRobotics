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


%% Trajectory tracking
disp('Generating Trajectory ...');
trajectory = test_trajectory(start, stop, path, true, map, decomps, time_allocation);
disp('Blue line is Trajectory planning.');
disp('Red line is Trajectory tracking.');

%% Gif
% makeGifAndJpg(3);     %figure(3): Graph with trajectory
