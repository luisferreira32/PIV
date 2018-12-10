function [objects] = track3D_part1( imgseq1,   cam_params)
%% 1. Get our data ready

%for test use preliminary and imgseq is dir name, for project use loader
[film_length, imgsrt, imgsd]=loader(imgseq1, cam_params);
%[imgsrt, imgsd]=loader(imgseq1, cam_params);
% film_length = length(imgseq1);
info = "done loading"

%% 2. Now box objects moving
% we extract object boxes and points clouds to further use

[image_objects] = extract_objects(film_length, imgsd);
info = "done extracting"

%% 3. Identify which objects are the same, creating a path between boxes
% connect boxes paths using proximity of boxes (and possible size)
% then use colour from imgsrt, (lastly use features)

objects = get_path(film_length, image_objects, imgsrt, imgsd);
info = "done tracking"
% test it graphically?

end



   



