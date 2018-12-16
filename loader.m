function [fl, imgsrt, imgsd] = loader( imgseq, cam_params)
    % Make use of the arguments

    % empty images rgb, rgb R & T, depth
    imgs=zeros(480,640,3,length(imgseq));
    imgsrt=zeros(480,640,3,length(imgseq));
    imgsd=zeros(480,640,length(imgseq));

    % film length
    fl = length(imgseq);

    % Load information and compute digital RGB camera
    for i=1:length(imgseq)
        d=dir(imgseq(i).rgb);
        dd=dir(imgseq(i).depth);
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
        imgsrt(:,:,:,i)=aux;
        
        % finally save our depth array for further use
        imgsd(:,:,i)=double(depth_array)/1000;
        
    end

end