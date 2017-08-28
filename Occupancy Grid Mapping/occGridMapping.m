% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1) ranges: this is the distance (d) from robot to the cell hit by Lidar,
% NxK where N is the number of measurements, and K is the number of scans
% at each time step. 
%
% 2) scanAngles: the angles of each scan ray, Nx1, all the same at every
% time step, denoted as alpha.
%
% 3) pose: 3xK matrix, each column is [x, y, theta]', the pose of robot at
% each time step,and [x, y] is the relative position to robot's
% original map position, but not the map position.
% 
% 4) param: parameters for grid and for log_odd
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters 
% 
% the number of grids in metric map for 1 meter in real world
myResol = param.resol;

% The initial map array size in pixels. For this PA, no need to worry about
% invalid range of measurement going far out of map, so this is fixed.
myMap = zeros(param.size);

% the origin (x, y) of robot in the map array, is used only for updating
% the log-odd values in the metric grid map.
 myOrigin = param.origin; 

%  Log-odd parameters 
 log_odd_occ = param.lo_occ;
 log_odd_free = param.lo_free; 
 log_odd_max = param.lo_max;
 log_odd_min = param.lo_min;
 
 N = length(scanAngles);    % number of rays at each time step
 K = size(pose,2);          % number of time steps
 for k = 1:K                % for each time step,
 % Find grids hit by the rays, 
    % pose of robot in real world (position in meters and yaw in radiant)
    x = pose(1, k);
    y = pose(2, k);
    theta = pose(3, k);
    
    % local coordinate of occupied points in real world, a Nx1 vector,
    % given the definition of body frame of robot
    x_occ = ranges(:,k) .* cos((scanAngles + theta)) + x;
    y_occ = -ranges(:,k) .* sin((scanAngles + theta)) + y;
    
    % coordinate of robot in metric map (i.e.,its pose is matched to grid),
    % which is the starting point of Bresenham Line.
    grid_x = ceil(x * myResol);
    grid_y = ceil(y * myResol);
   
    for n = 1: N        % for each ray, 
        % the coordinate of each occupied cell in metric grid map
       grid_occ = ceil(myResol * ([x_occ(n), y_occ(n)]));
       
       % the corodinate of free cells  between the current pose and the
       % occupied cell in metric grid map hit by the ray - Bresenham Line
       [freex, freey] = bresenham(grid_x, grid_y, grid_occ(1),grid_occ(2) );
       
       % convert coordinate to serial index to array accessed by [y, x]
       free = sub2ind(size(myMap),freey+myOrigin(2),freex+myOrigin(1));
       occ = sub2ind(size(myMap), grid_occ(2)+myOrigin(2), grid_occ(1)+myOrigin(1));

        % update the log-odd on metric grid map
        myMap(free) = myMap(free) - log_odd_free;
        myMap(occ) = myMap(occ) + log_odd_occ;       
    end
 end
 % Saturate the log-odd values
myMap(myMap < log_odd_min) = log_odd_min;
myMap(myMap > log_odd_max) = log_odd_max;
end

