clear;
dir_str = "datasets/bonecos/";
load("cameraparametersAsus.mat")

% do structs
% load all the images and mat
imgjpg = strcat(dir_str,'rgb_image1*.png');
imgmat = strcat(dir_str, 'depth1*.mat');
d=dir(imgjpg);
dd=dir(imgmat);
% pass it to a struct
imgseq1(length(d)) = struct();
for i = 1:length(d)
    imgseq1(i).rgb = fullfile(d(i).folder,d(i).name);
    imgseq1(i).depth = fullfile(dd(i).folder,dd(i).name);
end
%%
objects = track3D_part1(imgseq1,cam_params);
[fl, imgs, imdepth] = loader(imgseq1, cam_params);

%%
for i = 1:length(objects)
    figure(i)
	for  k = 1:length(objects(i).frames_tracked)
        clf;
        imshow(uint8(imgs(:,:,:,objects(i).frames_tracked(k))));
        hold on
		%plot3(objects(i).X(k,:), objects(i).Y(k,:), objects(i).Z(k,:))
		plot(objects(i).X(k,:), objects(i).Y(k,:))
        pause(0.1);
    end
end