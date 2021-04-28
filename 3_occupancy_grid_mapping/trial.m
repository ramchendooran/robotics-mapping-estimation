myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);

for j = 1:3*N/4 % for each time,
    
      
    % Find grids hit by the rays (in the gird map coordinate)
    x1_occ = ranges(:,j).*cos(pose(3,j)+scanAngles) + pose(1,j);
    x2_occ = -ranges(:,j).*sin(pose(3,j)+scanAngles) + pose(2,j);
    i_1k = ceil(myResol.*x1_occ) + myorigin(1);
    i_2k = ceil(myResol.*x2_occ) + myorigin(2);
    pos_x = ceil(myResol.*pose(1,j)) + myorigin(1);
    pos_y = ceil(myResol.*pose(2,j)) + myorigin(2);
    % Find occupied-measurement cells and free-measurement cells
    for i = 1:size(i_1k,1)       
        [freex, freey] = bresenham(pos_x,pos_y,i_1k(i),i_2k(i));
        free = sub2ind(size(myMap),freey,freex);
        % Update the log-odds
        myMap(i_2k(i), i_1k(i)) = myMap(i_2k(i), i_1k(i)) + param.lo_occ;
        myMap(free) = myMap(free) - param.lo_free;
    end
    
end
myMap(myMap > lo_max) = lo_max;
myMap(myMap < lo_min) = lo_min;
figure(1),
imagesc(myMap)
colormap('gray');
axis equal;