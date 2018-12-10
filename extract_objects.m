function [image_objects, image_pcs] = extract_objects(film_length, imgsrt, imgsd, cam_params)

% create labels' arrays
imglabel = zeros(480, 640, film_length);
imgnum = zeros(film_length);

% find the background with median ON DEPTH
bgdepth=median(imgsd,3);

% show results, to be commented
%figure(3);
%imagesc(bgdepth);   

% pre alocate structs
image_objects(film_length) = struct();
image_pcs(film_length) = struct();

% use the depth image to check objects moving (the best)
for i=1:film_length
    % subtract background
    imdiff=abs(imgsd(:,:,i)-bgdepth)>0.2;
    % filter image, maybe another filter to get better objects?
    imgdiffiltered=imopen(imdiff,strel('disk',6));
        
    % check with gradients between overlapping objects
    [Gmag, ~] = imgradient(imgsd(:,:,i));
    lin_indexes = Gmag > 1;
    imgdiffiltered(lin_indexes) = 0;
    
    % label every object
    [L, num]=bwlabel(imgdiffiltered);
            
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
        
        % compute point cloud and store it for further use
        pc = get_object_pc(pixel_list, imgsrt(:,:,:,i), imgsd(:,:,i), cam_params);
        % get values from point cloud
        [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(pc);
        
        image_pcs(i).object{p} = pc;
        % DEBUG
        %showPointCloud(pc);
        %pause(0.1)
        
        % set up the final object struct
        image_objects(i).object(p).X = [xmin, xmin, xmin, xmin, xmax, xmax, xmax, xmax];
        image_objects(i).object(p).Y = [ymax, ymax, ymin, ymin, ymax, ymax, ymin, ymin];
        image_objects(i).object(p).Z = [zmax, zmin, zmin, zmax, zmax, zmin, zmin, zmax];
        image_objects(i).object(p).frames_tracked = i;
        
        % next object
        p = p +1;
    end
end

end
