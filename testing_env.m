clear;
dir_str = "datasets/um/";
load("cameraparametersAsus.mat")

% do structs
% load all the images and mat
imgjpg1 = strcat(dir_str,'rgb_image1*.png');
imgmat1 = strcat(dir_str, 'depth1*.mat');
d1=dir(imgjpg);
dd1=dir(imgmat);
imgjpg2 = strcat(dir_str,'rgb_image2*.png');
imgmat2 = strcat(dir_str, 'depth2*.mat');
d2=dir(imgjpg);
dd2=dir(imgmat);
% pass it to a struct
imgseq1(length(d1)) = struct();
imgseq2(length(d1)) = struct();
for i = 1:length(d1)
    imgseq1(i).rgb = fullfile(d1(i).folder,d1(i).name);
    imgseq1(i).depth = fullfile(dd1(i).folder,dd1(i).name);
    imgseq2(i).rgb = fullfile(d2(i).folder,d2(i).name);
    imgseq2(i).depth = fullfile(dd2(i).folder,dd2(i).name);
end

%%
[objects, cam2toW] = track3D_part2(imgseq1,imgseq2,cam_params);
%%
[film_length, imgsrt1, imgsd1]=loader(imgseq1, cam_params);
[film_length, imgsrt2, imgsd2]=loader(imgseq2, cam_params);
Kd = cam_params.Kdepth;
for i = 1:length(objects)
    figure(i)
    for  k = 1:length(objects(i).frames_tracked)
        clf;
        [v, u]=ind2sub([480 640],(1:480*640));
        depth_array = imgsd1(:,:,objects(i).frames_tracked(k));
        Z=double((depth_array(:))');
        P=inv(Kd)*[Z.*u ;Z.*v;Z];
        pc=pointCloud(P', 'color',uint8(reshape(imgsrt1(:,:,:,objects(i).frames_tracked(k)), [480*640, 3])));
        showPointCloud(pc);
        hold on
        plot3(objects(i).X(k,:), objects(i).Y(k,:), objects(i).Z(k,:))
        pause(0.1);
    end
end