function [imgsrt, imgsd, imagewdepth] = preliminary( imgseq1,   cam_params)
%% 1. Get our pointclounds ready

% Make use of the arguments

% load params
load(cam_params)
% empty images rgb, rgb R & T, depth
imgs=zeros(480,640,3,length(imgseq1));
imgsrt=zeros(480,640,3,length(imgseq1));
imgsd=zeros(480,640,length(imgseq1));
% empty image with depth
imagewdepth.im = zeros(480,640,3,length(imgseq1));

% Load information and compute digital RGB camera
for i=1:length(imgseq1)
    d=dir(imgseq1(i).rgb);
    dd=dir(imgseq1(i).depth);
    % load the information
    imgs(:,:,:,i)=imread(fullfile(d.folder,d.name));
    load(fullfile(dd.folder,dd.name));
    Z=double(depth_array(:)')/1000;
    
    % compute correspondence
    [v, u]=ind2sub([480 640],(1:480*640));
    P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
    niu=cam_params.Krgb*[cam_params.R cam_params.T]*[P;ones(1,640*480)];
    u2=round(niu(1,:)./niu(3,:));
    v2=round(niu(2,:)./niu(3,:));
    
    % and compute new image
    im=imread(fullfile(d.folder,d.name));
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
