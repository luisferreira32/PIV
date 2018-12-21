%% Load data
%clear;
dir_str = "datasets/fruta1/";
load("cameraparametersAsus.mat")

% do structs
% load all the images and mat
imgjpg1 = strcat(dir_str,'rgb_image1*.png');
imgmat1 = strcat(dir_str, 'depth1*.mat');
d1=dir(imgjpg1);
dd1=dir(imgmat1);
imgjpg2 = strcat(dir_str,'rgb_image2*.png');
imgmat2 = strcat(dir_str, 'depth2*.mat');
d2=dir(imgjpg2);
dd2=dir(imgmat2);
%% Images selected
samples=fix(0.20*length(d1));
img_index=fix(rand(samples,1)*length(d1))+1;

for i=1:length(img_index)
    imgseq1(i).rgb = fullfile(d1(img_index(i)).folder,d1(img_index(i)).name);
    imgseq1(i).depth = fullfile(dd1(img_index(i)).folder,dd1(img_index(i)).name);
    imgseq2(i).rgb = fullfile(d2(img_index(i)).folder,d2(img_index(i)).name);
    imgseq2(i).depth = fullfile(dd2(img_index(i)).folder,dd2(img_index(i)).name);
end


%% Load
[~, rgbd1, dp1] = loader( imgseq1, cam_params);
[~, rgbd2, dp2] = loader( imgseq2, cam_params);

%% Compute 3d in camera frame
[v, u]=ind2sub([480 640],(1:480*640));
for i=1:length(img_index)
    % cam 1
    Z = dp1(:,:,i);
    Z = Z(:)';
    P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
    xyz1(:,:,i) = P';
    % cam 2
    Z = dp2(:,:,i);
    Z = Z(:)';
    P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
    xyz2(:,:,i) = P';
end

%% Perform SIFT for all images 
P1=[];
P2=[];

for i=1:length(img_index)
    I1=single(rgb2gray(uint8(rgbd1(:,:,:,i))));
    I2=single(rgb2gray(uint8(rgbd2(:,:,:,i))));
    [f1,d1_]=vl_sift(I1);
    [f2,d2_]=vl_sift(I2);
    [matches, scores] = vl_ubcmatch(d1_, d2_);
    y1=round(f1(2,matches(1,:)));
    x1=round(f1(1,matches(1,:)));
    y2=round(f2(2,matches(2,:)));
    x2=round(f2(1,matches(2,:)));
    ind1=sub2ind(size(dp2(:,:,i)),y1,x1);
    ind2=sub2ind(size(dp2(:,:,i)),y2,x2);
    p1=xyz1(ind1,:,i);
    p2=xyz2(ind2,:,i);
    final_inds=find((p1(:,3).*p2(:,3))>0);
    p1=p1(final_inds,:);p2=p2(final_inds,:);
    P1=[P1; p1];
    P2=[P2; p2];
end

%% Fabricated data to test ransac
[s,~, d]= svd(magic(3));
Rfab = s*d;
if det(Rfab) < 0
    Rfab = Rfab.*(-1);
end
Tfab = [1 0 1];
xyz2fab1 = xyz1(:,:,1)*Rfab + ones(length(xyz1(:,:,1)),1)*transform.c(1,:);

%choose "features" matches
P1=[];
P2=[];
for i=1:(fix(length(xyz2fab1)/1000))
    p1 = xyz1(i,:,1);
    p2 = xyz2fab1(i,:);
    P1 = [P1; p1];
    P2 = [P2; p2];
end
final_inds=find((P1(:,3).*P2(:,3))>0);
P1=P1(final_inds,:);P2=P2(final_inds,:);

%% Ransac
n_it=1000; % ??
threshold=0.4;
num_in=[];
% Generate random numbers
n_points=fix(rand(4*n_it,1)*length(P1))+1;
espera=1;

for i=1:n_it-3
    % Choose random points, we need at least 4 points for the procrustes
    rand_points=n_points(4*i:4*i+3);
    % Get 3D points correspondent to theses indexes
   
    X1=P1(rand_points,:);
    X2=P2(rand_points,:);
    
    % Compute procrustes
    [~, ~, transform]=procrustes(X1,X2, 'scaling', false, 'reflection', false);
    
    % Compute points from image 2 projected to image 1 using the
    % transformation from procrustes
    
    xyz21= P2 * transform.T + ones(length(P2),1)*transform.c(1,:);
    error=sqrt(sum((P1-xyz21).^2,2)); % Compute the error for all points
    
    inds(i).in=find(error<threshold);
    
    % Add the number of inliers to the inliers vector
    
    num_in=[num_in length(inds(i).in)];
   
    % Waitbar
    waitbar(espera/(n_it-3))
    espera=espera+1;
    
end
%%
[~, ind]=max(num_in);
xyz1f=P1(inds(ind).in,:);
xyz2f=P2(inds(ind).in,:);
[~, ~, transform]=procrustes(xyz1f, xyz2f, 'scaling', false, 'reflection', false);
R=transform.T;
T=transform.c(1,:)';

%% Test
pc1 = pointCloud(xyz1(:,:,1), 'color', reshape(uint8(rgbd1(:,:,:,1)), [640*480, 3]));
figure(1);clf;showPointCloud(pc1);
% pc2 = pointCloud(xyz2(:,:,1), 'color', reshape(uint8(rgbd2(:,:,:,1)), [640*480, 3]));
% figure(2);showPointCloud(pc2);
xyz2rt1 = xyz2(:,:,1)*R + ones(length(xyz2(:,:,1)),1)*transform.c(1,:);
pc3 = pointCloud(xyz2rt1, 'color', reshape(uint8(rgbd2(:,:,:,1)), [640*480, 3]));
figure(3);clf;showPointCloud(pc3);
pc_merged = pcmerge(pc1,pc3,0.01);
figure(4);clf;showPointCloud(pc_merged);
