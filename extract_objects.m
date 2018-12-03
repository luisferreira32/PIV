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
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.20;
    % filter image, maybe another filter to get better objects?
    imgdiffiltered=imopen(imdiff,strel('disk',7));
    
    % label every object
    [L, num]=bwlabel(imgdiffiltered);
        
    % show results (to be commented)
    %figure(4);
    %imagesc(imgdiffiltered);
    %colormap(gray);
    %figure(5);
    %imagesc(bwlabel(imgdiffiltered));
    %pause(0.01);
    
    % then box it
    p = 1;
    for j = 1:num
        % check indexes for each label and compute 2d extremes
        [rows, columns] = find(L == j);
        pixel_list = [rows, columns];
        % check if area is at least 500 pixels
        if length(pixel_list(:,1)) < 500
            continue
        end
        
        % compute point cloud and store it for further use
        pc = get_object_pc(pixel_list, imgsrt(:,:,:,i), imgsd(:,:,i), cam_params);
        % get values from point cloud
        [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(pc);
        
        image_pcs(i).object{p} = pc;
        
        % set up the final object struct
        image_objects(i).object(p).X = [xmin, xmin, xmin, xmin, xmax, xmax, xmax, xmax];
        image_objects(i).object(p).Y = [ymax, ymax, ymin, ymin, ymax, ymax, ymin, ymin];
        image_objects(i).object(p).Z = [zmax, zmin, zmin, zmax, zmax, zmin, zmin, zmax];
        image_objects(i).object(p).frames_tracked = i;
        
        %figure(1);showPointCloud(pc);
        % next object
        p = p +1;
    end
end

end
