function [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(imgd, pixel_list)
% Input: 
% PC: computed points in 3D with size 3xnumpoints
% Output:
% boxpoints: vector with the points that form the bounding boz around the
% object in the point cloud 

% ERROR POSSIBILITY
xmin=min(pixel_list(:,1));
ymin=min(pixel_list(:,2));
zmin=imgd(min(pixel_list(:,1)),min(pixel_list(:,2)));
xmax=max(pixel_list(:,1));
ymax=max(pixel_list(:,2));
zmax=imgd(max(pixel_list(:,1)),max(pixel_list(:,2)));

end