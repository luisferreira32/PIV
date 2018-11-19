function [imgsrt, imgsd, imagewdepth] = preliminary( imgseq1,   cam_params)
%% 1. Get our pointclounds ready

% Make use of the arguments

% load params
load(cam_params)
%get names & create empty arrays
jpg = "*.jpg";
mat = "*.mat";
imgjpg = strcat(imgseq1, jpg);
imgmat = strcat(imgseq1, mat);
d=dir(imgjpg);
dd=dir(imgmat);
% empty images rgb, rgb R & T, depth
imgs=zeros(480,640,3,length(d));
imgsrt=zeros(480,640,3,length(d));
imgsd=zeros(480,640,length(d));
% empty image with depth
imagewdepth.im = zeros(480,640,3,length(d));

% Load information and compute digital RGB camera
for i=1:length(d)
    % load the information
    imgs(:,:,:,i)=imread(fullfile(d(i).folder,d(i).name));
    load(fullfile(dd(i).folder,dd(i).name));
    Z=double(depth_array(:)')/1000;
    
    % compute correspondence
    [v, u]=ind2sub([480 640],(1:480*640));
    P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
    niu=cam_params.Krgb*[cam_params.R cam_params.T]*[P;ones(1,640*480)];
    u2=round(niu(1,:)./niu(3,:));
    v2=round(niu(2,:)./niu(3,:));
    
    % and compute new image
    im=imread(fullfile(d(i).folder,d(i).name));
    im2=zeros(640*480,3);
	indsclean=find((u2>=1)&(u2<=641)&(v2>=1)&(v2<=480));
	indscolor=sub2ind([480 640],v2(indsclean),u2(indsclean));
	im1aux=reshape(im,[640*480 3]);
	im2(indsclean,:)=im1aux(indscolor,:);
    aux=uint8(reshape(im2,[480,640,3]));
    imagewdepth.im(:,:,:,i)=aux; 
    imgsrt(:,:,:,i)=aux;
    
    % finally save our depth array for further use
    imgsd(:,:,i)=double(depth_array)/1000;
    
    % show results (to be commented)
    %figure(1)
    %imshow(uint8(imgs(:,:,:,i)));
    %figure(2)
    %imshow(uint8(imgsrt(:,:,:,i)));
    %pause(0.01);
    
end

end