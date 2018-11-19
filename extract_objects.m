function [image_objects] = extract_objects(size, imgsrt, imgsd, imagewdepth)

% create labels' arrays
imglabel = zeros(480, 640, size);
imgnum = zeros(size);

% find the background with median ON DEPTH
bgdepth=median(imgsd,3);

% show results, to be commented
%figure(3);
%imagesc(bgdepth);   

% use the depth image to check objects moving (the best)
for i=1:size
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

% choice note: FOR NOW IT'S NOT OPTIMAL BOX (just a vertical one).
% The top points are 1,2,3,4 clockwise starting at 12 o'clock,
% and bottom points are 5,6,7,8 clockwise starting at 12 o'clock
% FUTURE: use point cloud here along with label to separate objects and
% encapsulate them

% box each object on each image
image_objects(size) = struct();

% this way we create a struct we can re-use to build the final objects
for i = 1:size
    image_objects(i).object(imgnum(i)) = struct();
    for j = 1:imgnum(i)
        % check indexes for each label and compute 2d extremes
        [rows, columns] = find(imglabel(:,:,i) == j);
        % check if area is at least 500 pixels
        if size(rows) < 500
            continue
        end
        
        % X and Y are min and max of rows labelled
        image_objects(i).object(j).X = [min(rows), min(rows), min(rows), min(rows), max(rows), max(rows), max(rows), max(rows)];
        image_objects(i).object(j).Y = [max(columns), max(columns), min(columns), min(columns), max(columns), max(columns), min(columns), min(columns)];
        % Z we check in the depth image within object label
        zsmall = min(imgsd(min(rows):max(rows),min(columns):max(columns),i));
        zbig = max(imgsd(min(rows):max(rows),min(columns):max(columns),i));
        image_objects(i).object(j).Z = [zbig, zsmall, zsmall, zbig, zbig, zsmall, zsmall, zbig];
        image_objects(i).object(j).frames_tracked = i;
    end
end

end