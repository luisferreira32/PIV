function [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(pc)
% Input: 
% PC: computed points in 3D with size 3xnumpoints
% Output:
% boxpoints: vector with the points that form the bounding boz around the
% object in the point cloud 

xmin=min(pc(:,1));
xmax=max(pc(:,1));
ymin=min(pc(:,2));
ymax=max(pc(:,2));
zmin=min(pc(:,3));
zmax=max(pc(:,3));

end
