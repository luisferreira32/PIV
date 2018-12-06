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

<<<<<<< HEAD
xmin=min(pc(:,1));
xmax=max(pc(:,1));
ymin=min(pc(:,2));
ymax=max(pc(:,2));
zmin=min(pc(:,3));
zmax=max(pc(:,3));
boxpoints=zeros(8,3);
boxpoints(1,1:3)=[xmin, ymin, zmin];
boxpoints(2,1:3)=[xmax, ymin, zmin];
boxpoints(3,1:3)=[xmin, ymax, zmin];
boxpoints(4,1:3)=[xmax, ymax, zmin];
boxpoints(5,1:3)=[xmin, ymin, zmax];
boxpoints(6,1:3)=[xmax, ymin, zmax];
boxpoints(7,1:3)=[xmin, ymax, zmax];
boxpoints(8,1:3)=[xmax, ymax, zmax];
end
=======
end
>>>>>>> 8c3033f477b96618428d8943e227698d26273e56
