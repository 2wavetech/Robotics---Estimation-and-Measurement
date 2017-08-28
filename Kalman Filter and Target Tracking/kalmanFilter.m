function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y
%   (x, y): sensor data
%   state: state vector at previous time step
%   param: it has only one field param.P, which is the estimation error
%   covariance
% (predictx, predicty) is the projected position 10 time steps from
% t into future

    %% Place parameters like covarainces, etc. here:
    P = eye(4);         % covariance of the dynamic system, 4x4
    R = eye(2);         % covariance of the measurement, 2x2
    P(1,1) = 1;         % set position variance x to 0.1 while velocity variance remains 1
    P(2,2) = 1;         % same for y
    P(3,3) = 10;
    P(4,4) = 10;
    R = R * 1;          % set measurement variance of position to 0.1
    
    dt = t - previous_t;         % one time step is 30ms
    
    % build up the state transition matrix for dynamics system, 4x4
    A = [1 0 dt 0; 
         0 1 0  dt;
         0 0 1  0; 
         0 0 0  1];

     % build up the measurement model matrix
     C = [1 0 0 0; 
          0 1 0 0];
     
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 1000 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end
    
    previous_x = state(1);
    previous_y = state(2);
    
    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    % vx = (x - state(1)) / (t - previous_t);
    % vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    % predictx = x + vx * 0.330;
    % predicty = y + vy * 0.330;
    % State is a four dimensional element
    % state = [x, y, vx, vy];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Prediction
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    estimate_state = A * state';        % 4x1
    estimate_P = A * param.P * A' + P;  % 4x4
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    zk = C * [x; y; 0; 0];          % measurement, C2x4, true state = 4x1 => 2x1
    Yk = zk - C * estimate_state;   % measurement residual, 2x1
    Sk = C * estimate_P * C' + R;   % residual covariance, 2x2
    Kk = estimate_P * C' /Sk;       % optimal Kalman gain, 4x2
    state = estimate_state + Kk * Yk;   % state update, 4x1
    param.P = estimate_P - Kk * C * estimate_P;    % covariance update
    
    % Even though observation does not cover velocity, state transition can
    % update velocity, which is interesting.
    
    state = state';                     % turn state to 1x4
 
    predictx = state(1) + state(3)*10*dt;                % x
    predicty = state(2) + state(4)*10*dt;                % y

end
