close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path 1
% disp('Path Planning ...');
map = load_map('maps/ellipsoid.txt', 1, 1, 0.25);
start = {[15  2 2]};
% stop  = {[15  20 19]};
stop  = {[5  20 19]};

nquad = length(start);
for qn = 1:nquad
    disp('JPS time is :');
    tic
    path{qn} = jump_point_search_3D(map, start{qn}, stop{qn});
    % path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
    path{qn} = [start{qn}; path{qn}(1:end,:)];
    path{qn}(end + 1,:) = stop{qn};
    toc
end

    % delete the points of no-use
    path{1} = simplify_path2(map, path{1});
   
    % turn id to points of obj
    obs = [];
    [x, y, z] = size(map.occ_map);
    for i = 1 : x
        for j = 1 : y
            for k = 1 : z
                if map.occ_map(i, j, k) == 1
                    p1 = idx_to_points(map, [i j k]);
                    obs(end+1, :) = p1;
                end
            end
        end
    end
    
    %% call SFC
    disp('SFC time is :');
    tic
    decomp = SFC_3D(path{1}, obs);
    toc
    
if nquad == 1
    plot_path(map, path{1}, decomp); %draw path and blocks
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% draw_ObcPoints
%draw_ObcPoints


%% Trajectory planning
disp('Generating Trajectory ...');
trajectory_generator([], [], map, path, decomp);

%% Trajectory tracking
trajectory = test_trajectory(start, stop, map, path, true, decomp); % with visualization
disp('Blue line is Trajectory planning.');
disp('Red line is Trajectory tracking.');

%% Gif
% makeGifAndJpg






