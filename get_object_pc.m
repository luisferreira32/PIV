function [pc]=get_object_pc(pixel_list, imgrgb,imgdepth, R21, T21, cam_params)

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

% compute xyz
Z=double(depth_index(:)');
u=pixel_list(:,2)';
v=pixel_list(:,1)';
P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
% rotate and translate, T21 horizontal
xyz = (P')*R21 + repmat(T21, length(P'), 1);
pc=pointCloud(xyz,'color',uint8(rgb_index));

end