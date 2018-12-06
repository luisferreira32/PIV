<<<<<<< HEAD
function [pc]=get_object_pc(pixel_list, imgrgb,imgdepth,bgdepth)
=======
function [pc]=get_object_pc(pixel_list, imgrgb,imgdepth, cam_params)
>>>>>>> 8c3033f477b96618428d8943e227698d26273e56
% Inputs: 
% pixel_list- list of pixels that belong to the same object
% imgrgb- rgb image obtained from the reconstruction 
% imgdepth- depth image corresponding to the rgb image with background
% subtraction
%
% Outputs:
% pc - point cloud obtained from the depth and rgb image

imdiff=abs(imgdepth(:,:,10)-bgdepth)>.25;

rgb_index=zeros(length(pixel_list),3);
<<<<<<< HEAD
depth_index=zeros(length(pixel_list,1));
=======
depth_index=zeros(length(pixel_list),1);
>>>>>>> 8c3033f477b96618428d8943e227698d26273e56

% Get the pixel values from the rgb image 
for i=1:length(pixel_list)
    rgb_index(i,:)=imgrgb(pixel_list(i,1),pixel_list(i,2),:);
end

% Get the pixel values from the depth image 

for i=1:length(pixel_list)
<<<<<<< HEAD
    depth_index(i)=imdiff(pixel_list(i,2),pixel_list(i,1));
=======
    depth_index(i)=imgdepth(pixel_list(i,1),pixel_list(i,2));
>>>>>>> 8c3033f477b96618428d8943e227698d26273e56
end

Z=double(depth_index(:)');
u=pixel_list(:,2)';
v=pixel_list(:,1)';
P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
pc=pointCloud(P','color',uint8(rgb_index));

end