%% Load data
load cameraparametersAsus.mat
%load calib_asus.mat
d1=dir('/Users/catiafortunato/Documents/MATLAB/piv_project/proj2/fruta3/depth1*.mat');
d2=dir('/Users/catiafortunato/Documents/MATLAB/piv_project/proj2/fruta3/depth2*.mat');
dd1=dir('/Users/catiafortunato/Documents/MATLAB/piv_project/proj2/fruta3/rgb_image1*.png');
dd2=dir('/Users/catiafortunato/Documents/MATLAB/piv_project/proj2/fruta3/rgb_image2*.png');
%% Images selected
samples=fix(0.20*length(d1));
img_index=fix(rand(samples,1)*length(d1))+1;

for i=1:length(img_index)
    img1(:,:,:,i)=imread(dd1(img_index(i)).name);
end
for i=1:length(img_index)
    img2(:,:,:,i)=imread(dd2(img_index(i)).name);
end

%% Depth image

for i=1:length(img_index)
    load(d1(img_index(i)).name)
    dp1(:,:,i)=depth_array;
end
for i=1:length(img_index)
    load(d2(img_index(i)).name)
    dp2(:,:,i)=depth_array;
end

%% Get intrinsic parameters

K=cam_params.Kdepth;

%% Compute projection coordinates in camera frame

XYZ1=[];
for i=1:length(img_index)
    dp=dp1(:,:,i);
    xyz1= get_xyzasus(dp(:),[480 640],1:640*480,K,1,0);
    XYZ1=[XYZ1 xyz1];
end

XYZ2=[];
for i=1:length(img_index)
    dp=dp2(:,:,i);
    xyz2= get_xyzasus(dp(:),[480 640],1:640*480,K,1,0);
    XYZ2=[XYZ2 xyz2];
end

%% Register RGB to depth 

for i=1:length(img_index)
    rgbd1(:,:,:,i)= get_rgbd(XYZ1(:,i:i+2), img1(:,:,:,i), cam_params.R, cam_params.T, cam_params.Krgb);
end

for i=1:length(img_index)
    rgbd2(:,:,:,i)= get_rgbd(XYZ2(:,i:i+2), img2(:,:,:,i), cam_params.R, cam_params.T, cam_params.Krgb);
end

%% Perform SIFT for all images 
P1=[];
P2=[];

for i=1:length(img_index)
    I1=single(rgb2gray(rgbd1(:,:,:,i)));
    I2=single(rgb2gray(rgbd2(:,:,:,i)));
    [f1,d1]=vl_sift(I1);
    [f2,d2]=vl_sift(I2);
    [matches, scores] = vl_ubcmatch(d1, d2);
    y1=round(f1(2,matches(1,:)));
    x1=round(f1(1,matches(1,:)));
    y2=round(f2(2,matches(2,:)));
    x2=round(f2(1,matches(2,:)));
    ind1=sub2ind(size(dp2),y1,x1);
    ind2=sub2ind(size(dp2),y2,x2);
    p1=XYZ1(ind1,i:i+2);
    p2=XYZ2(ind2,i:i+2);
    P1=[P1; p1];
    P2=[P2; p2];
end

%% Ransac
n_it=1000; % ??
threshold=0.005;
num_in=[];
trans=[];
% Generate random numbers
n_points=fix(rand(4*n_it,1)*length(P1))+1;

espera=1;

for i=1:n_it-3
    % Choose random points, we need at least 4 points for the procrustes
    rand_points=n_points(4*i:4*i+3);
    %ind1=sub2ind(size(dp2),round(f1(2,rand_points(1,:))),round(f1(1,rand_points(1,:))));
    %ind2=sub2ind(size(dp2),round(f2(2,rand_points(2,:))),round(f2(1,rand_points(2,:))));
    % Get 3D points correspondent to theses indexes
   
    X1=P1(rand_points,:);
    X2=P2(rand_points,:);
    
    % Compute procrustes
    [d, Z, transform]=procrustes(X1,X2);
    
    % Compute points from image 2 projected to image 1 using the
    % transformation from procrustes
    
    xyz21= transform.b * P2 * transform.T + repmat(transform.c(1,:),length(P2),1);
    error=sqrt(sum((P1-xyz21).^2,2)); % Compute the error for all points
    
    inds(i).in=find(error<0.1);
    
    % Add the number of inliers to the inliers vector
    
    num_in=[num_in length(inds(i).in)];
    % Save the transformation in each iteration
    trans=[trans transform];
   
    % Waitbar
    waitbar(espera/(n_it-3))
    espera=espera+1;
    
end
%%
[m, ind]=max(num_in);
xyz1f=P1(inds(ind).in,:);
xyz2f=P2(inds(ind).in,:);
[d, T, transform]=procrustes(xyz1f, xyz2f);
R=transform.T;
T=transform.c(1,:);
