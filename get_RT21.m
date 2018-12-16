function [R21, T21] = get_RT21(film_length, imgsrt1, imgsrt2, imgsd1, imgsd2, cam_params)
	% call this function N times, depending on film_length
	[R21, T21] = single_RT(rgbd1, rgbd2, imgsd1, imgsd2, cam_params);
end