function [image_objects] = extract_objects(film_length, imgsd)

% find the background with median ON DEPTH
bgdepth=median(imgsd,3);

% show results, to be commented
%figure(3);
%imagesc(bgdepth);   

% pre alocate structs
image_objects(film_length) = struct();

% use the depth image to check objects moving (the best)
for i=1:film_length    
    % subtract background
    imdiff=abs(imgsd(:,:,i)-bgdepth)>.20;
    % filter image, maybe another filter to get better objects?
    imgdiffiltered=imopen(imdiff,strel('disk',6));
    
    % label every object
    [L, num]=bwlabel(imgdiffiltered);
    % check with gradients between overlapping objects
    [Gmag, ~] = imgradient(imgsd(:,:,i));
    lin_indexes = Gmag > 1;
    imgdiffiltered(lin_indexes) = 0;
        
    % DEBUG
    imagesc(imgsd(:,:,i))
    
    % then box it
    p = 1;
    for j = 1:num
        % check indexes for each label and compute 2d extremes
        [rows, columns] = find(L == j);
        pixel_list = [rows, columns];
        % check if area is at least 1000 pixels
        if length(pixel_list(:,1)) < 1000
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
        
        %DEBUG
        hold on
        plot(image_objects(i).object(p).X, image_objects(i).object(p).Y)
        
        % next object
        p = p +1;
    end
    %DEBUG
    pause(0.1)
    hold off
end

end
