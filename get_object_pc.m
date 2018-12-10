function [pc]=get_object_pc(pixel_list, imgrgb,imgdepth, cam_params)
% Inputs: 
% pixel_list- list of pixels that belong to the same object
% imgrgb- rgb image obtained from the reconstruction 
% imgdepth- depth image corresponding to the rgb image 
%
% Outputs:
% pc - point cloud obtained from the depth and rgb image

rgb_index=zeros(length(pixel_list),3);
depth_index=zeros(length(pixel_list),1);

% Get the pixel values from the rgb image 
for i=1:length(pixel_list)
    rgb_index(i,:)=imgrgb(pixel_list(i,1),pixel_list(i,2),:);
end

% Get the pixel values from the depth image 

for i=1:length(pixel_list)
    depth_index(i)=imgdepth(pixel_list(i,1),pixel_list(i,2));
end

Z=double(depth_index(:)');
u=pixel_list(:,2)';
v=pixel_list(:,1)';
P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
pc=pointCloud(P','color',uint8(rgb_index));

end