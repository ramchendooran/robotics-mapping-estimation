% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

imagepath = 'C:\Users\ramch\Documents\Matlab\Robotics\Estimation and LEarning\Week 1\train';
Samples = [];
for k=1:15
    % Load image
    I = imread(sprintf('%s/%03d.png',imagepath,k));
    
    % You may consider other color space than RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), 
    mask = roipoly(I); 
    figure(2), imshow(mask); title('Mask');
    sample_ind = find(mask > 0);
    
    R = R(sample_ind);
    G = G(sample_ind);
    B = B(sample_ind);
    
    Samples = [Samples; [R G B]];
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
end
%%
% visualize the sample distribution
figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%
Samples_double = double(Samples);
mean_yb = mean(Samples,1);
cov_yb = cov(Samples_double);
var_A = var(Samples_double,1);
%%
im1 = imread(sprintf('%s/%03d.png',imagepath,2));
imshow(im1);
im1 = double(im1);
prob = zeros([size(im1,1), size(im1,2)]);
for i = 1:size(im1,1)
    for j = 1:size(im1,2)
        prob(i,j) = (1/(((2*pi)^1.5)*(det(cov_yb)^0.5)))*exp((-0.5)*(reshape(im1(i,j,:), [3,1])-mean_yb')'*inv(cov_yb)*(reshape(im1(i,j,:), [3,1])-mean_yb'));
    end
end
figure;
surf(prob)
bw = 1*(prob>1e-5);

figure, imshow(bw);
%%
% create new empty binary image
bw_biggest = false(size(bw));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
figure,
imshow(bw_biggest); hold on;
% show the centroid
% http://www.mathworks.com/help/images/ref/regionprops.html
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
plot(loc(1), loc(2),'r+');
