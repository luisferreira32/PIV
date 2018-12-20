% Load data
clear;
dir_str = "datasets/fruta1/";
load("cameraparametersAsus.mat")
%run('~/Documents/MATLAB/vlfeat-0.9.21/toolbox/vl_setup');

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
    Z1 = dp1(:,:,i);
    Z1 = Z1(:)';
    P1=inv(cam_params.Kdepth)*[Z1.*u ;Z1.*v;Z1];
    xyz1(:,:,i) = P1';
    % cam 2
    Z2 = dp2(:,:,i);
    Z2 = Z2(:)';
    P2=inv(cam_params.Kdepth)*[Z2.*u ;Z2.*v;Z2];
    xyz2(:,:,i) = P2';
end

%% Perform SIFT for all images 
P1=[];
P2=[];
espera=1;
for i=1:length(img_index)
    I1=single(rgb2gray(uint8(rgbd1(:,:,:,i))));
    I2=single(rgb2gray(uint8(rgbd2(:,:,:,i))));
    [f1,d1_]=vl_sift(I1);
    [f2,d2_]=vl_sift(I2);
    
%     figure(1);
%     imagesc(rgbd1(:,:,:,i));hold on;plot(f1(1,:),f1(2,:),'*');hold off;
%     figure(2);
%     imagesc(rgbd2(:,:,:,i));hold on;plot(f2(1,:),f2(2,:),'*');hold off;

    [matches, scores] = vl_ubcmatch(d1_, d2_, 3);
    xy1=f1(1:2,matches(1,:));
    xy2=f2(1:2,matches(2,:));
    
    % get indexes of rgb equivalent on depth
    iaux1=cam_params.Krgb*xyz1(:,:,i)';
    iaux2=cam_params.Krgb*xyz2(:,:,i)';
    ind1=[iaux1(1,:)./iaux1(3,:); iaux1(2,:)./iaux1(3,:)];
    ind2=[iaux2(1,:)./iaux2(3,:); iaux2(2,:)./iaux1(3,:)];
    % and pass to linear indexes the matches through minimum between rgb
    % index and match index.
    for j = 1:size(xy1,2)
        [~,bestxy1(j)]=min(sqrt(sum(abs( ind1 - repmat( xy1(:,j),1,length(ind1) ) ).^2,1)));
        [~,bestxy2(j)]=min(sqrt(sum(abs( ind2 - repmat( xy2(:,j),1,length(ind2) ) ).^2,1)));
    end

%     % the "old" way
%     ind1=sub2ind([480 640],round(xy1(2,:)),round(xy1(1,:)));
%     ind2=sub2ind([480 640],round(xy2(2,:)),round(xy2(1,:)));
    
    p1=xyz1(bestxy1,:,i);
    p2=xyz2(bestxy2,:,i);
    P1=[P1; p1];
    P2=[P2; p2];
    
    waitbar(espera/length(img_index));
    espera = espera +1 ;
end

%% Fabricated data to test ransac
% [s,~, d]= svd(magic(3));
% Rfab = s*d;
% if det(Rfab) < 0
%     Rfab = Rfab.*(-1);
% end
% Tfab = [1 0 1];
% xyz2fab1 = xyz1(:,:,1)*Rfab + repmat(Tfab, length(xyz1(:,:,1)), 1);
% 
% %choose "features" matches
% P1=[];
% P2=[];
% for i=1:(fix(length(xyz2fab1)/1000))
%     random_index = fix(i*1000*rand())+1;
%     P1 = [P1; xyz1(random_index,:,1)];
%     P2 = [P2; xyz2fab1(random_index,:)];
% end
% 
% % add noise of cm to 50%
% for i = 1:(fix(length(P1)*0.5))
%     random_index = fix((i*2*rand()))+1;
%     P1(random_index,3) = P1(random_index,3) + rand()*0.2;
%     P2(random_index,3) = P2(random_index,3) + rand()*0.2;
% end

%% Ransac
n_it=1000; % ??
threshold=0.01;
num_in=[];
trans=[];
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
    
    xyz21= P2 * transform.T + repmat(transform.c(1,:),length(P2),1);
    error=sqrt(sum((P1-xyz21).^2,2)); % Compute the error for all points
    
    inds(i).in=find(error<threshold);
    
    % Add the number of inliers to the inliers vector
    
    num_in=[num_in length(inds(i).in)];
    % Save the transformation in each iteration
    trans=[trans transform];
   
    % Waitbar
    waitbar(espera/(n_it-3))
    espera=espera+1;
    
end
%%
[~, ind]=max(num_in);
xyz1f=P1(inds(ind).in,:);
xyz2f=P2(inds(ind).in,:);
[~, ~, transform]=procrustes(xyz1f, xyz2f,'scaling', false, 'reflection', false);
R=transform.T';
T=transform.c(1,:)';

%% Test it
pc1 = pointCloud(xyz1(:,:,1), 'color', reshape(uint8(rgbd1(:,:,:,1)), [640*480, 3]));
 figure(1);showPointCloud(pc1);
% pc2 = pointCloud(xyz2(:,:,1), 'color', reshape(uint8(rgbd2(:,:,:,1)), [640*480, 3]));
% figure(2);showPointCloud(pc2);
xyz2rt1 = xyz2(:,:,1)*R' + repmat(T', length(xyz2(:,:,1)), 1);
pc3 = pointCloud(xyz2rt1, 'color', reshape(uint8(rgbd2(:,:,:,1)), [640*480, 3]));
 figure(3);showPointCloud(pc3);
pc_merged = pcmerge(pc1,pc3,0.01);
figure(4);showPointCloud(pc_merged);

% test fabricated data
% xyz2fab1rt = xyz2fab1*R' + repmat(T', length(xyz2fab1),1);
% pc4 = pointCloud(xyz2fab1rt, 'color', reshape(uint8(rgbd1(:,:,:,1)), [640*480, 3]));
% figure();showPointCloud(pcmerge(pc1,pc4, 0.001));


