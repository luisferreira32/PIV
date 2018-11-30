imgseq1 = "datasets/um/images";
cam_params_str = "cameraparametersAsus.mat";

objects = track3D_part1(imgseq1,cam_params_str);
[fl, imgs, imdepth] = preliminary(imgseq1, cam_params_str);
load(cam_params_str);
Kd = cam_params.Kdepth;
%%
for i = 1:length(objects)
    figure(i)
	for  k = 1:length(objects(i).frames_tracked)
        [v, u]=ind2sub([480 640],(1:480*640));
        depth_array = imdepth(:,:,objects(i).frames_tracked(k));
        Z=double((depth_array(:))');
        P=inv(Kd)*[Z.*u ;Z.*v;Z];
        pc=pointCloud(P', 'color',uint8(reshape(imgs(:,:,:,objects(i).frames_tracked(k)), [480*640, 3])));
        showPointCloud(pc);
        hold on
		plot3(objects(i).X(k,:), objects(i).Y(k,:), objects(i).Z(k,:))
		pause(.5)
        clf;
	end
end