%% SETTING UP THE STUFF
clear;
imgseq1_str = "datasets/um/images";
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
[fl, imgs, imgsd] = loader(imgseq1, cam_params);
bgdepth=median(imgsd,3);

%%
for i=1:fl
    % subtract background
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.20;
    % filter image, maybe another filter to get better objects?
    imgdiffiltered=imopen(imdiff,strel('disk',6));
    
    % GRADIENT HERE %
    [Gmag, ~] = imgradient(imgsd(:,:,i));
    lin_indexes = Gmag > 1;
    %grad_filt = zeros(480,640);
    %grad_filt(lin_indexes) = 1;
    %figure(1)
    %imshow(imgdiffiltered);
    imgdiffiltered(lin_indexes) = 0;
    %figure(2)
    %imshowpair(imgdiffiltered, grad_filt, 'montage');    
    % label every object
    [L, num]=bwlabel(imgdiffiltered);
    num
        
    % DEBUG
    imagesc(imgsd(:,:,i));
    colormap(gray);
    
    % then box it
    p = 1;
    for j = 1:num
        % check indexes for each label and compute 2d extremes
        [rows, columns] = find(L == j);
        pixel_list = [rows, columns];
        % check if area is at least 3000 pixels ~5%
        if length(pixel_list) < 3000
            continue
        end
        
        % get values from depth
        [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(imgsd(:,:,i), pixel_list);
               
        % set up the final object struct
        image_objects(i).object(p).X = [xmin, xmin, xmin, xmin, xmax, xmax, xmax, xmax];
        image_objects(i).object(p).Y = [ymax, ymax, ymin, ymin, ymax, ymax, ymin, ymin];
        image_objects(i).object(p).Z = [zmax, zmin, zmin, zmax, zmax, zmin, zmin, zmax];
        image_objects(i).object(p).frames_tracked = i;
        
        %and add the pixel list!
        image_objects(i).object(p).pixel_list = pixel_list;
        % DEBUG
        hold on
        plot( image_objects(i).object(p).X,  image_objects(i).object(p).Y)
        
        % next object
        p = p +1;
    end
    % DEBUG
    pause(0.1)
    hold off
end