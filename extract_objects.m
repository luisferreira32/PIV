function [image_objects, image_pcs] = extract_objects(film_length, imgsrt, imgsd, cam_params)

% create labels' arrays
imglabel = zeros(480, 640, film_length);
imgnum = zeros(film_length);

% find the background with median ON DEPTH
bgdepth=median(imgsd,3);

% show results, to be commented
%figure(3);
%imagesc(bgdepth);   

% use the depth image to check objects moving (the best)
for i=1:film_length
    % subtract background
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.20;
    % filter image, maybe another filter to get better objects?
    imgdiffiltered=imopen(imdiff,strel('disk',6));
    
    % label every object
    [L, num]=bwlabel(imgdiffiltered);
    imglabel(:,:,i) = L;
    imgnum(i) = num;
        
    % show results (to be commented)
    %figure(4);
    %imagesc(imgdiffiltered);
    %colormap(gray);
    %figure(5);
    %imagesc(bwlabel(imgdiffiltered));
    %pause(0.01);
end


% box each object on each image
image_objects(film_length) = struct();
image_pcs(film_length) = struct();

% this way we create a struct we can re-use to build the final objects
for i = 1:film_length
    p = 1;
    for j = 1:imgnum(i)
        % check indexes for each label and compute 2d extremes
        [rows, columns] = find(imglabel(:,:,i) == j);
        pixel_list = [rows, columns];
        % check if area is at least 500 pixels
        if length(pixel_list(:,1)) < 500
            continue
        end
        
        % compute point cloud and store it for further use
        pc = get_object_pc(pixel_list, imgsrt(:,:,:,i), imgsd(:,:,i), cam_params);
        image_pcs(i).object{p} = pc;
        
        % THIS BOXING SHOULD BE WITH THE POINT CLOUD
        % X and Y are min and max of rows labelled
        image_objects(i).object(p).X = [min(pixel_list(:,1)), min(pixel_list(:,1)), min(pixel_list(:,1)), min(pixel_list(:,1)), max(pixel_list(:,1)), max(pixel_list(:,1)), max(pixel_list(:,1)), max(pixel_list(:,1))];
        image_objects(i).object(p).Y = [max(pixel_list(:,2)), max(pixel_list(:,2)), min(pixel_list(:,2)), min(pixel_list(:,2)), max(pixel_list(:,2)), max(pixel_list(:,2)), min(pixel_list(:,2)), min(pixel_list(:,2))];
        % Z we check in the depth image within object label
        zsmall = min(imgsd(min(pixel_list(:,1)):max(pixel_list(:,1)),min(pixel_list(:,2)):max(pixel_list(:,2)),i));
        zbig = max(imgsd(min(pixel_list(:,1)):max(pixel_list(:,1)),min(pixel_list(:,2)):max(pixel_list(:,2)),i));
        image_objects(i).object(p).Z = [zbig, zsmall, zsmall, zbig, zbig, zsmall, zsmall, zbig];
        image_objects(i).object(p).frames_tracked = i;
        
        % next object
        p = p +1;
    end
end

end