function [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(pc)
% Input: 
% PC: computed points in 3D with size 3xnumpoints
% Output:
% boxpoints: vector with the points that form the bounding boz around the
% object in the point cloud 
xmin=min(pc.Location(:,1));
xmax=max(pc.Location(:,1));
ymin=min(pc.Location(:,2));
ymax=max(pc.Location(:,2));
zmin=min(pc.Location(:,3));
zmax=max(pc.Location(:,3));

end