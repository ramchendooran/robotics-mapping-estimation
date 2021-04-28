function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    P = eye(4);
    Q = eye(4)*0.5;
    R = eye(2)*0.1;

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    vx = (x - state(1)) / (t - previous_t);
    vy = (y - state(2)) / (t - previous_t);
    dt = t - previous_t;
    % Predict 330ms into the future
    predictx = x + vx * 0.330;
    predicty = y + vy * 0.330;
    
    % A matrix 
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    
    % Predicted state  
    x_check = [predictx; predicty; vx; vy];
    
    % Propogate state covariance
    P_check = A*P*A' + Q;
    
    % Measured value 
    C = [1 0 0 0; 0 1 0 0];
    z  = C*[x; y; vx; vy];
    
    % Compute Kalman gain
    K = P_check*C'*inv(R + C*P_check*C');
    
    % Compute Corrected mean 
    x_hat = x_check + K*(z - C*x_check);
    
    % Compute corrected variance
    param = P_check - K*C*P_check;
    
    % State is a four dimensional element
    state = x_hat;
    predictx = x_hat(1);
    predicty = x_hat(2);
end
