% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array
mean_y = [147.1104  143.0098   63.7302];
cov_y = [209.1431  128.3725 -219.9078; 128.3725  132.5009 -168.1882; -219.9078 -168.1882  369.0137];
im1 = I;
im1 = double(im1);
prob = zeros([size(im1,1), size(im1,2)]);
for i = 1:size(im1,1)
    for j = 1:size(im1,2)
        prob(i,j) = (1/(((2*pi)^1.5)*(det(cov_y)^0.5)))*exp((-0.5)*(reshape(im1(i,j,:), [3,1])-mean_y')'*inv(cov_y)*(reshape(im1(i,j,:), [3,1])-mean_y'));
    end
end

bw = 1*(prob>1e-5);


% create new empty binary image
bw_biggest = false(size(bw));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(bw);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
segI = bw_biggest;
% show the centroid
% http://www.mathworks.com/help/images/ref/regionprops.html
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
% thre = 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
