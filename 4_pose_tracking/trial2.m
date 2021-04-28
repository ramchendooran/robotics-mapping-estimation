load practice.mat
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
Ma = 100;                           % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, Ma]);
clear x1_idx;
clear x1_occ;
clear x2_idx;
clear x2_occ;
% Weight initialization
w = 1/Ma.*ones([1 Ma]);
% Initialize motion model noise parameters
mu = zeros(1, 3);
sig = diag([0.01, 0.01, 0.01]);
%%
for j = 2:1000 % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 
%     mu = [0 0 0];
%     sigma = diag([0.05 0.05 0.008]);
%     R = chol(sigma);
%     z = (repmat(0,Ma,1) + randn(Ma,3)*R)';
    z = mvnrnd(mu, sig, Ma)';
    P = P + z;
    x1_occ = zeros([1081 Ma]);
    x2_occ = zeros([1081 Ma]);
    x1_idx = zeros([1081 Ma]);
    x2_idx = zeros([1081 Ma]);
    corr = zeros([1 Ma]);
    % 2) Measurement Update 
    % For each particle
    for i = 1:Ma
        %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
        x1_occ(:,i) = ranges(:,j).*cos(scanAngles + P(3,i)) + P(1,i);
        x2_occ(:,i) = (-1)*(ranges(:,j)).*sin(scanAngles + P(3,i)) + P(2,i);
        x1_idx(:,i) = ceil(myResolution.*x1_occ(:,i)) + myOrigin(1);
        x2_idx(:,i) = ceil(myResolution.*x2_occ(:,i)) + myOrigin(2);
%         figure(1),
%         imagesc(M); hold on;
%         plot(x1_idx(:,i), x2_idx(:,i), 'g.');
        corr(i) = 0;
        
        for k = 1:size(x1_occ(:,i))             
            if(x2_idx(k,i) > size(M,1) || x1_idx(k,i)< 1 || x1_idx(k,i) > size(M,2) || x2_idx(k,i) < 1)
                continue
                %corr(i) = corr(i) + 0.49;
                % score(k,i) = 0.49;
            elseif(M(x2_idx(k,i), x1_idx(k,i)) >= -0.2 && M(x2_idx(k,i), x1_idx(k,i)) < 0.5)
                continue
            elseif M(x2_idx(k,i), x1_idx(k,i)) >= 0.5
                corr(i) = corr(i) + 10;
                % corr(i) = corr(i) + M(x2_idx(k,i), x1_idx(k,i));
                % score(k,i) = M( x2_idx(k,i), x1_idx(k,i));
            elseif M(x2_idx(k,i), x1_idx(k,i)) < -0.2
                corr(i) = corr(i) -5;
            end
        end
    end
        %   2-2) For each particle, calculate the correlation scores of the particles
        
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
%     w = 1/Ma.*ones([1 Ma]);
%     P = repmat(myPose(:,j), [1 Ma]);
    w = w_m;
    P = P_m;
    % 4) Visualize the pose on the map as needed
    figure(1),
    imagesc(M); hold on;

    % Plot LIDAR data
    lidar_global(:,1) = ceil(myResolution.*(ranges(:,j).*cos(scanAngles + myPose(3,j)) + myPose(1,j)))+ myOrigin(1);
    lidar_global(:,2) = ceil(myResolution.*((-1)*(ranges(:,j)).*sin(scanAngles + myPose(3,j)) + myPose(2,j)))+ myOrigin(2);

    plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 
    
    colormap('gray');
    axis equal;
    hold on;
    plot(myPose(1,:)*myResolution+myOrigin(1), ...
        myPose(2,:)*myResolution+myOrigin(2), 'r.-');

end