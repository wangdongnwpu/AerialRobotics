function [ desired_state ] = trajectory_generator(t, qn, map, path, decomp)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;
persistent map0 path0 total_time X ts;
if numel(t) == 0 | numel(qn) == 0
   map0 = map;
   path0 = path;
    % speed = 2;
    % speed = 5.25;  % is cloest plan time == sim time
    speed = 4;
    disp(['The planned speed is : ', num2str(speed)]);
   [ts, total_time] = generate_ts(path0{1}, speed);
   disp(['time management: total_time is ', num2str(total_time), 'seconds']);
   disp(['Split time is : ', num2str(ts)]);
   tic
   %  use 'close form' get Trajectory planning. ===========================
   % X = traj_opt7(path0{1}, total_time,ts);
   
   % use 'Quadratic Programming' get Trajectory planning. ========================
   % but x,y,z separate solution. 
   % [X, path0{1}, ts] = quadraticProgramming(path0{1}', total_time);
   
   % use 'Quadratic Programming' get Trajectory planning. ========================
   % use SFC make Inequality constraints
   X = QPbyUseSFC(path0{1}', ts, decomp);
   disp('generator trajectory time is :');
   toc
   return
end

if nargin < 4
    map = map0;
    path = path0;
end

p = path{qn};
if t >= total_time
    pos = p(end,:);
    vel = [0;0;0];
    acc = [0;0;0];
else
%     
%     3rd order trajectory planning
%     k = find(ts<=t);
%     k = k(end);
%     pos = [t^3, t^2, t, 1]*X(4*(k-1)+1:4*k,:);
%     vel = [3*t^2, 2*t, 1, 0]*X(4*(k-1)+1:4*k,:);
%     acc = [6*t, 2, 0, 0]*X(4*(k-1)+1:4*k,:);

%     % 7th order minimum snap trajectory
    k = find(ts<=t);
    k = k(end);
    pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*X(8*(k-1)+1:8*k,:);
    vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*X(8*(k-1)+1:8*k,:);
    acc = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0]*X(8*(k-1)+1:8*k,:);
end

yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
