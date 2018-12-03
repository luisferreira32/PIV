clear;
imgseq1_str = "datasets/confusao/images";
load("cameraparametersAsus.mat")

% do structs
% load all the jpg and mat
jpg = "*.jpg";
mat = "*.mat";
imgjpg = strcat(imgseq1_str, jpg);
imgmat = strcat(imgseq1_str, mat);
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

Kd = cam_params.Kdepth;
%%
for i = 1:length(objects)
    figure(i)
	for  k = 1:length(objects(i).frames_tracked)
        clf;
        [v, u]=ind2sub([480 640],(1:480*640));
        depth_array = imdepth(:,:,objects(i).frames_tracked(k));
        Z=double((depth_array(:))');
        P=inv(Kd)*[Z.*u ;Z.*v;Z];
        pc=pointCloud(P', 'color',uint8(reshape(imgs(:,:,:,objects(i).frames_tracked(k)), [480*640, 3])));
        showPointCloud(pc);
        hold on
		plot3(objects(i).X(k,:), objects(i).Y(k,:), objects(i).Z(k,:))
        pause(0.01);
    end
    % for now just show the first X objects
    if i > 1
        break;
    end
end