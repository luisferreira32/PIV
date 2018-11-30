imgseq1 = "datasets/um/images";
cam_params = "cameraparametersAsus.mat";

objects = track3D_part1(imgseq1,cam_params);
[fl, imgs, imdepth] = preliminary(imgseq1, cam_params);

for i = 1:length(objects)
	for  k = 1:length(objects(i).frames_tracked)
		figure(1)
		%imshow(uint8(imgs(:,:,:,objects(i).frames_tracked(k))))
        hold on
		plot(objects(i).X(k,:), objects(i).Y(k,:), '*')
		pause(.5)
	end
end