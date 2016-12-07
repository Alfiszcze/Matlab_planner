%% ROS Quadrotor Planner using PRM's
clear all;
local_waypoint_pub_1  = rospublisher('mavros_custom_path1','std_msgs/Float32MultiArray');
local_pos_sub_1  = rossubscriber('/copter1/local_position/pose', @poseCallback);
global_quadrotor_sub_1 = rossubscriber('/copter1/global_position/global', @gpsCallback);
%global_obstacale_sub_1 = rossubscriber('/gui/obstacle', @obstacleCallback)
global_goal_sub_1 = rossubscriber('/gui/goal1', @goalCallback)
%global_goal_sub_2 = rossubscriber('/gui/goal2', @goalCallback)

global pose
global orient
global obstacle
global home
global goal1

home_location = [];
obstacle_location = [];
goal_location = [];
obstacle = []
prm = robotics.PRM;
prm.NumNodes = 2000;
prm.ConnectionDistance = 2;
scale = 0.01;

% System parameters.

goal1 = [];
robotRadius = 0.5;
path_recompute_count = 50;
ros_rate = 20.0;
map_size = 10;



% For timing purposes.
r = rosrate(ros_rate);
prev_time = r.TotalElapsedTime;
i = 0;

% Begin by arming the Quadrotor using arming service
%testreq = rosmessage(ros_arming_service)
%testresp = call(ros_arming_service, testreq, 'Timeout', 3);
scale_factor = 100;
scale_matrix = eye(2)*scale_factor;
path_generated = [];

% Loop to calculate paths indefinitely.
while true
   % obstacle_location
    
    
% Initialize home location once.
if (size(home_location,1)==0 && size(home,1) > 0)
    [x_h y_h] = deg2utm(home(1), home(2));
    home_location = [x_h y_h];
end

if (size(obstacle,1)>0)
    obstacle_location = home_location - deg2utm(obstacle(1),obstacle(2));
end
if (size(goal1,1)>0)
    [x_g y_g] = deg2utm(goal1(1),goal1(2));
    goal_location = [x_g y_g] - home_location;
end
i= i + 1;

%%
map = robotics.BinaryOccupancyGrid(map_size,map_size,5);
bias = [map_size/2 map_size/2];


if (size(obstacle_location,1) > 0)
    setOccupancy(map,obstacle_location*inv(scale_matrix) +bias,1);
end




mapInflated = copy(map);
inflate(mapInflated,robotRadius);

prm.Map = mapInflated;
%show(mapInflated)
% Recompute Path only once every ten cycles.
if (mod(i,path_recompute_count) ==0 && size(pose,1) > 0 && size(goal_location,1) > 0)
    
    startLocation = pose(1:2)*inv(scale_matrix)+bias
endLocation = goal_location*inv(scale_matrix) + bias
    fprintf('Iteration: %d - Time Elapsed: %f\n',i,r.TotalElapsedTime - prev_time)
    prev_time = r.TotalElapsedTime;
    path_generated = findpath(prm, startLocation, endLocation);
    if (size(path_generated,1) > 0)
    path_generated = (path_generated - repmat(bias, size(path_generated,1),1))*scale_matrix
    end
    show(prm)
end

if length(path_generated) >0
    msg = rosmessage(local_waypoint_pub_1);
    msg.Data=[path_generated(:,1); path_generated(:,2)];
   % msg.Pose.Position.X = path_generated(1,1)-bias(1);
   % msg.Pose.Position.Y = path_generated(1,1)-bias(1);
   % msg.Pose.Position.Z = 2;

    send(local_waypoint_pub_1,msg)
end

waitfor(r);
end
