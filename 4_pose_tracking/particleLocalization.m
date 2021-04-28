% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)


% Number of poses to calculate
N = size(ranges, 2);

% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.


% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ma = 1000;       % Computer Savu ;)   % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, Ma]);
% Weight initialization
w = 1/Ma.*ones([1 Ma]);
% Initialize motion model noise parameters
mu = zeros(1, 3); % 0 mean

% Set std by trial and error
% If too high, robot position will diverge
% If too low, robot will not move
sig = diag([0.04, 0.04, 0.007]); 

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 
    z = mvnrnd(mu, sig, Ma)'; % random motion model noise
    P = P + z;
    corr = zeros([1 Ma]);
    range = ranges(:,j);
    
    % 2) Measurement Update 
    % For each particle
    for i = 1:Ma
        %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
        
        % Pose of the particle in meter ( From myOrigin frame)
        x1 = P(1,i);
        x2 = P(2,i);
        theta = P(3,i);
        
        % Calculate Lidar hits in meter ( myOrigin frame)
        x1_occ = range.*cos(scanAngles + theta) + x1;
        x2_occ = (-1)*(range).*sin(scanAngles + theta) + x2;
        
        % Calculate Lidar hits in index ( grid map coordinate frame)
        x1_idx = ceil(myResolution.*x1_occ) + myOrigin(1);
        x2_idx = ceil(myResolution.*x2_occ) + myOrigin(2);
        
        % Pose of the robot in index ( grid map coordinate frame)
        % Used to calculate free cells by Bresenhams
        % x1_rob = ceil(x1*myResolution) + myOrigin(1);
        % x2_rob = ceil(x2*myResolution) + myOrigin(2);
        
        % Create a mask to remove invalid indices
        idx = (x1_idx > 1 & x1_idx < size(map,2) & x2_idx > 1 & x2_idx < size(map,1));
        
        % Remove invalid indices ( Now both dimensions have to be same)
        x1_idx = x1_idx(idx);
        x2_idx = x2_idx(idx);
        
        clear idx;
        % Convert x1_idx, x2_idx into 1D indices 
        % This way, we don't have to use for loop for correlation addition
        idx = sub2ind(size(map),x2_idx,x1_idx);
        
        % Initialize correlation 
        corr(i) = 0;
        
        % Calculate correlation
        N = map(idx);
        corr(i) = corr(i) + sum(N(N>0.5)*10);
        corr(i) = corr(i) + sum(N(N<-0.2)*-5);
    end
   
        
    %   2-3) Update the particle weights         
    corr = corr - min(corr);
    w = w.*corr;
    w = w/ sum(w);
    
    %   2-4) Choose the best particle to update the pose
    [w, idx] = sort(w);
    P(:,:) = P(:,idx);
    cum = cumsum(w);
    myPose(:,j) = P(:,end);
    % 3) Resample if the effective number of particles is smaller than a threshold
    rv = rand([1 Ma]);
    P_m = zeros([3 Ma]);
    w_m = zeros([1 Ma]);
    for i = 1:size(rv,2)
        element = find(cum>rv(i));
        w_m(i) = w(element(1));
        P_m(:,i) = P(:,element(1));
    end
    % Reset weights
    w = 1/Ma.*ones([1 Ma]);
    % Make all poses as best pose (duplicates will be removed due to noise)
    P = repmat(myPose(:,j), [1 Ma]);
    
    % Actually have to use this but not working
%     w = w_m;
%     P = P_m;
    % 4) Visualize the pose on the map as needed
    figure(1),
    imagesc(map); hold on;

%     Plot LIDAR data
    lidar_global(:,1) = ceil(myResolution.*(ranges(:,j).*cos(scanAngles + myPose(3,j)) + myPose(1,j)))+ myOrigin(1);
    lidar_global(:,2) = ceil(myResolution.*((-1)*(ranges(:,j)).*sin(scanAngles + myPose(3,j)) + myPose(2,j)))+ myOrigin(2);

    plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 
    
    colormap('gray');
    axis equal;
    hold on;
    plot(myPose(1,:)*myResolution+myOrigin(1), ...
        myPose(2,:)*myResolution+myOrigin(2), 'r.-');

end

end


