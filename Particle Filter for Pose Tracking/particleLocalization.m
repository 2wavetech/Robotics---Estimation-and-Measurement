% Robotics: Estimation and Learning 
% WEEK 4 - Particle Filter for Robot Localization
% Jiyang Wang
% Refer to the instruction for the details of the input parameters.
function myPose = particleLocalization(ranges, scanAngles, map, param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Map Parameters 
% 
% the number of grids for 1 meter.
 myResol = param.resol;
% the origin of the map in pixels
 myOrigin = param.origin; 
 
% The initial pose is given. You should put the given initial pose into
% myPose for j=1, ignoring the j=1 ranges. The pose(:,1) should be the pose
% when ranges(:,j) were measured.
 myPose(:,1) = param.init_pose;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Number of poses to calculate - N is the total number of time steps
N = size(ranges, 2);

% Output format is [x1 x2, ...; y1, y2, ...; theta1, theta2, ...]
myPose = zeros(3, N);

dimension = size(myPose, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Build up the transition model which is the distribution of odometry that
% models the movements of robot. The particle filter samples from this
% distribution many times to update each particle¡¯s position and
% orientation (i.e., the pose of robot = {x, y, theta}). state = {x, y,
% theta, Vx, Vy, Vtheta}.

dt = 0.03;                    % one time step is 30ms
    
% state transition matrix for dynamics model, 6x6, used for Kalman Filter
% to project the next position of robot.
    A = [1 0 0 dt 0 0; 
         0 1 0 0 dt 0;
         0 0 1 0 0  dt; 
         0 0 0 1 0  0;
         0 0 0 0 1  0;
         0 0 0 0 0  1];
% noise to odometry measurement, used for Kalman Filter
    OdometryVar = eye(6);           % covariance of dynamic system, 6x6
    OdometryVar(1,1) = 1;           % set x variance
    OdometryVar(2,2) = 1;           % set y variance
    OdometryVar(3,3) = 0.1;         % set theta variance
    OdometryVar(4,4) = 0.1;         % set x velocity variance
    OdometryVar(5,5) = 0.1;         % set y velocity variance
    OdometryVar(6,6) = 0.05;        % set theta velocity variance    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Instead of Kalman Filter, we can use a simple dynamic model to project
% the next pose of robot by moving robot randomly with normal distribution
% around the current pose. Note that normal distribution is assumed in the
% absence of the knowledge of the actual distribution from which the
% particles should be drawn.
    mu = [0, 0, 0];                 % mean for (x, y, theta)
    variance = 0.05;                  % variance must be tuned by experimenting practice data
    var_theta = 0.1;
%     sigma = variance * eye(dimension);      % variance for (x, y, theta)
    sigma = [variance, 0, 0;
             0, variance, 0;
             0, 0, var_theta];
    % R = chol(sigma);                % Cholesky factorization: R'R = sigma

% system noise covariance for the simple model - not seems to be useful
    noise_variance = [0.02, 0, 0; 0, 0.02, 0; 0, 0, 0.01];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Decide the number of particles, M, which muse be tuned based on the
% experiments using the practice data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 400; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create M number of particles by just replicating myPose, which will
% become unique after random distribution is added.
OdomParticle = repmat(myPose(:,1), [1, M]);

% degree of correlation between LIDAR returns and map - I use a simple
% metric here by just counting the number of "occupied cells" on the
% postions of the map that are "hit" by laser. "free cells" correlation
% between LIDAR measurement and the map registration are not considered in
% order to save computing time.
corr_threshold = ceil(0.75 * size(scanAngles,1));     

% Occupancy value of the unexplored cells on the map is obtained by finding
% the log-odd value of the background cells in the map that appear with the
% highest frequency among all the cells. The occupied cells must have the
% log-odd value higher than this "background" value, while the free cells
% have much lower log-odd values. We use this value as a benchmark for
% deciding whether a cell is occupied or not.
unexplored = mode(reshape(map, size(map,1)*size(map,2), 1));

for j = 2:N    % start estimating myPose from j=2 using ranges(:,2).
%    M = size(OdomParticle, 2);      % the actual number of particles
    n_effective = ceil(M * 0.7);    % effective particle number expected
    weights = zeros(M, 1);          % initial weights of particles
    max_weight = 0;
    iter = 0;                       % for trouble shooting
    
    % Keep moving particles around until at least one of them has the
    % required correlation score higher than the threshold
    while max_weight < corr_threshold              
        % 1) particle movement - propagate the particles using simple dynamic
        %    model, assuming that the underlying distribution is normal distribution.
        system_variance = (randn(size(OdomParticle,2), dimension) * sigma)';
        OdomParticle = OdomParticle + system_variance;      % make a random movement to all the particles
     
        % add noise to each particle movement. Again, noise is Gaussian.
        %   system_noise = noise_variance * randn(dimension, M); 
        %   OdomParticle = OdomParticle + system_noise;
    
        % 2) Measurement Update 
    
        for k = 1:size(OdomParticle,2)                 % calculate correlation score for each particle ...
            
            % 2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
            %      pose of robot in real world (position in meters and yaw in radiant)
            Px = OdomParticle(1, k);    % a particle's predicted pose
            Py = OdomParticle(2, k);
            Ptheta = OdomParticle(3, k);
    
            % local coordinate of cells supposed to be hit by laser in real world, a Nx1 vector,
            % given the definition of body frame of robot
            occ_x = ceil(myResol * (ranges(:,j) .* cos(scanAngles + Ptheta) + Px)) + myOrigin(1);
            occ_y = ceil(myResol * (-ranges(:,j) .* sin(scanAngles + Ptheta) + Py)) + myOrigin(2);
            
            % exclude the corodinates that fall outside of the map
            valid_index = occ_x>0 & occ_y > 0 & occ_x<=size(map,2) & occ_y<=size(map,1);
  
            % 2-2) For each particle, calculate the correlation score
       
            % the map coordinates of the cells "hit" by laser rays - note
            % that x and y coordinate are used seperately, so it's better
            % to glue them by function sub2ind() when used to access the map.
            cells_occ = map(sub2ind(size(map), occ_y(valid_index), occ_x(valid_index)));

            %  2-3) Update the particle weights which is the number of
            %  occupied cells on map where the rays hit
            weights(k) = sum(cells_occ>unexplored);
%          ids = occ_x > 0 & occ_x <= size(map,2) & occ_y > 0 & occ_y <= size(map,1);
%          weights(k) = size( map( map (sub2ind(size(map), occ_y(ids), occ_x(ids)) ) > unexplored ), 1);

        end
        
        %  2-4) Choose the best particle to update the pose
        [max_weight, index_best] = max (weights);
        
        iter = iter+1;
        if mod(iter, 50) ==0
            fprintf('while iter=%d\n', iter);
        end
        if iter >=500
            disp('matching failed');
            break;
        end
    end
    
    % select the best particle as the pose
    myPose(:,j) = OdomParticle(:,index_best); 
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    Q = OdomParticle(:,weights>=corr_threshold);     % keep the particles that have good match
    if ~isempty(Q)
        OdomParticle = repmat(Q, 1, ceil(M/size(Q,2)) );
    end
 end
         
%     % 4) Visualize the pose on the map as needed
%    this part is in example_test.m

end

