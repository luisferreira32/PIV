run('/Users/catiafortunato/Downloads/vlfeat-0.9.21/toolbox/vl_setup')

%%
% Load data
load cameraparametersAsus.mat
%load calib_asus.mat
d1=dir('/Users/catiafortunato/Documents/MATLAB/piv_project/proj2/fruta3/depth1*.mat');
d2=dir('/Users/catiafortunato/Documents/MATLAB/piv_project/proj2/fruta3/depth2*.mat');
dd1=dir('/Users/catiafortunato/Documents/MATLAB/piv_project/proj2/fruta3/rgb_image1*.png');
dd2=dir('/Users/catiafortunato/Documents/MATLAB/piv_project/proj2/fruta3/rgb_image2*.png');
%% See images
image1=imread(dd1(25).name);
image2=imread(dd2(25).name);
  figure(1)
  imshow(image1)
  figure(2)
  imshow(image2)

%% Load depth image
load(d1(25).name);
dp1=depth_array;
load(d2(25).name);
dp2=depth_array;

%% Get intrinsic parameters

K=cam_params.Kdepth;

%% Compute projection coordinates in camera frame 
    
xyz1=get_xyzasus(dp1(:),[480 640],1:640*480,K,1,0); %points in image of camera 1 to points in camera frame
xyz2=get_xyzasus(dp2(:),[480 640],1:640*480,K,1,0); %points in image of camera 2 to points in camera frame

%% Compute coordinares in world coordinate frame
xyz1=xyz1*cam_params.R+repmat(cam_params.T',length(xyz1),1); 
xyz2=xyz2*cam_params.R+repmat(cam_params.T',length(xyz2),1);

%% Register RGB to depth 
rgbd1 = get_rgbd(xyz1, image1, cam_params.R, cam_params.T, cam_params.Krgb);
rgbd2 = get_rgbd(xyz2, image2, cam_params.R, cam_params.T, cam_params.Krgb);
imshow(uint8(reshape(rgbd1,[480,640,3])))

%% Compute point cloud
pc1=pointCloud(xyz1,'Color',reshape(rgbd1,[480*640 3]));
pc2=pointCloud(xyz2,'Color',reshape(rgbd2,[480*640 3]));

%% --------------------------------------------------------------
%% Choose image to get matching points - SIFT
I1=single(rgb2gray(rgbd1));
I2=single(rgb2gray(rgbd2));
[f1,d1]=vl_sift(I1);
[f2,d2]=vl_sift(I2);
[matches, scores] = vl_ubcmatch(d1, d2);
%% Get the index of the matches in the image 
y1=round(f1(2,matches(1,:)));
x1=round(f1(1,matches(1,:)));
y2=round(f2(2,matches(2,:)));
x2=round(f2(1,matches(2,:)));
ind1=sub2ind(size(dp2),y1,x1);
ind2=sub2ind(size(dp2),y2,x2);

%% Compute the 3D coordinates of the matches
P1=xyz1(ind1,:);
P2=xyz2(ind2,:);

%% Ransac
n_it=2000; % ??
threshold=0.005;
num_in=[];
trans=[];
% Generate random numbers
n_points=fix(rand(4*n_it,1)*length(matches))+1;

espera=1;

for i=1:n_it-3
    % Choose random points, we need at least 4 points for the procrustes
    rand_points=matches(:,n_points(4*i:4*i+3));
    ind1=sub2ind(size(dp2),round(f1(2,rand_points(1,:))),round(f1(1,rand_points(1,:))));
    ind2=sub2ind(size(dp2),round(f2(2,rand_points(2,:))),round(f2(1,rand_points(2,:))));
    
    % Get 3D points correspondent to theses indexes
    X1=xyz1(ind1,:);
    X2=xyz2(ind2,:);
    
    % Compute procrustes
    [d, Z, transform]=procrustes(X1,X2);
    
    % Compute points from image 2 projected to image 1 using the
    % transformation from procrustes
    
    xyz21= transform.b * P2 * transform.T + repmat(transform.c(1,:),length(P2),1);
    error=sqrt(sum((P1-xyz21).^2,2)); % Compute the error for all points
    
    inds(i).in=find(error<2);
    
    % Add the number of inliers to the inliers vector
    
    num_in=[num_in length(inds(i).in)];
    % Save the transformation in each iteration
    trans=[trans transform];
   
    % Waitbar
    waitbar(espera/(n_it-3))
    espera=espera+1;
    
end

%% Find the best result 

[m, ind]=max(num_in);
xyz1f=P1(inds(ind).in,:);
xyz2f=P2(inds(ind).in,:);
[d, T, transform]=procrustes(xyz1f, xyz2f);
R=transform.T;
T=transform.c(1,:);

