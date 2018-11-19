function [objects] = track3D_part1( imgseq1,   cam_params)
%% 1. Get our data ready

%for test use preliminary and imgseq is dir name, for project use loader
[imgsrt, imgsd, imagewdepth]=preliminary(imgseq1, cam_params);
%[ imgsrt, imgsd, imagewdepth]=loader(imgseq1, cam_params);
size = length(imgseq1);


%% 2. Now box objects moving

[image_objects, imglabel, imgnum] = extract_objects(size, imgsrt, imgsd, imagewdepth);




%% 3. Identify which objects are the same, creating a path between boxes
% connect boxes paths using proximity of boxes (and possible size)
% then use colour from imgsrt, (lastly use features)

objects = get_path(size, image_objects, imglabel, imgnum, imgsrt);


end



   



