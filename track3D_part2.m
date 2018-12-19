function [objects, cam2toW] = track3D_part2( imgseq1, imgseq2,   cam_params)
	%% 1. Get our data ready

	% load
	[~, imgsrt1, imgsd1]=loader(imgseq1, cam_params);
	[film_length, imgsrt2, imgsd2]=loader(imgseq2, cam_params);
	fprintf("done loading\n");
	% and do correspondence
	[R21, T21] = get_RT21(film_length, imgsrt1, imgsrt2, imgsd1, imgsd2, cam_params);
	cam2toW.R = R21;
	cam2toW.T = T21;

	%% 2. Now box objects moving
	[image_objects, image_pcs] = extract_objects(film_length, imgsrt1, imgsrt2, imgsd1, imgsd2, R21, T21, cam_params);
	fprintf("done extracting\n");

	%% 3. Identify which objects are the same, creating a path between boxes
	% gettin maximum volume and maximum norm to normalize the costs 0-1
	[maxvol, maxnorm] = get_maxes(film_length, imgsd1, cam_params);
	objects = get_path(film_length, image_objects, image_pcs, maxvol, maxnorm);
	fprintf("done tracking\n");

end
