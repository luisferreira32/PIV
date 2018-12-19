function [image_objects, image_pcs] = extract_objects(film_length, maxnorm, imgsrt1, imgsrt2, imgsd1, imgsd2, R21, T21,cam_params)


    % find the background with median ON DEPTH
    bgdepth1=median(imgsd1,3);
    bgdepth2=median(imgsd2,3);

    % pre alocate structs
    image_objects(film_length) = struct();
    image_pcs(film_length) = struct();
    image_objects2(film_length) = struct();
    image_pcs2(film_length) = struct();

    % use the depth image to check objects moving (the best)
    for i=1:film_length

        % FIRST CAMERA -- all objects here are final objects
        % subtract background
        imdiff=abs(imgsd1(:,:,i)-bgdepth1)>0.20;
        % filter image, maybe another filter to get better objects?
        imdifil=imopen(imdiff,strel('disk',6));
            
        % check with gradients between overlapping objects
        [Gmag, ~] = imgradient(imgsd1(:,:,i));
        lin_indexes = Gmag > 1;
        imdifil(lin_indexes) = 0;
        
        % label every object
        [L, num]=bwlabel(imdifil);
                
        % then box it
        object_num = 1;
        for j = 1:num
            % check indexes for each label and compute 2d extremes
            [rows, columns] = find(L == j);
            pixel_list = [rows, columns];
            % check if area is at least 3000 pixels ~5%
            if length(pixel_list) < 3000
                continue
            end        
            
            % compute point cloud and store it for further use
            R11 = [1,0,0; 0,1,0; 0,0,1];
            T11 = [0 0 0];
            pc = get_object_pc(pixel_list, imgsrt1(:,:,:,i), imgsd1(:,:,i), R11, T11, cam_params);
            % get values from point cloud
            [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(pc);
            
            image_pcs(i).object{object_num} = pc;
            
            % set up the final object struct
            image_objects(i).object(object_num).X = [xmin, xmin, xmin, xmin, xmax, xmax, xmax, xmax];
            image_objects(i).object(object_num).Y = [ymax, ymax, ymin, ymin, ymax, ymax, ymin, ymin];
            image_objects(i).object(object_num).Z = [zmax, zmin, zmin, zmax, zmax, zmin, zmin, zmax];
            image_objects(i).object(object_num).frames_tracked = i;
            
            % next object
            object_num = object_num +1;
        end
        curr_objects = object_num;


        % SECOND CAMERA
        % subtract background
        imdiff=abs(imgsd2(:,:,i)-bgdepth2)>0.20;
        % filter image, maybe another filter to get better objects?
        imdifil=imopen(imdiff,strel('disk',6));
            
        % check with gradients between overlapping objects
        [Gmag, ~] = imgradient(imgsd2(:,:,i));
        lin_indexes = Gmag > 1;
        imdifil(lin_indexes) = 0;
        
        % label every object
        [L, num]=bwlabel(imdifil);
                
        % then box it
        object_num = 1;
        for j = 1:num
            % check indexes for each label and compute 2d extremes
            [rows, columns] = find(L == j);
            pixel_list = [rows, columns];
            % check if area is at least 3000 pixels ~5%
            if length(pixel_list) < 3000
                continue
            end        
            
            % compute point cloud in world frame
            pc = get_object_pc(pixel_list, imgsrt2(:,:,:,i), imgsd2(:,:,i), R21, T21, cam_params);

            % get values from point cloud
            [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(pc);
            
            image_pcs2(i).object{object_num} = pc;
            
            % set up the final object struct
            image_objects2(i).object(object_num).X = [xmin, xmin, xmin, xmin, xmax, xmax, xmax, xmax];
            image_objects2(i).object(object_num).Y = [ymax, ymax, ymin, ymin, ymax, ymax, ymin, ymin];
            image_objects2(i).object(object_num).Z = [zmax, zmin, zmin, zmax, zmax, zmin, zmin, zmax];
            image_objects2(i).object(object_num).frames_tracked = i;
            
            % next object
            object_num = object_num +1;
        end


        % MATCH OBJECTS OF CAMERAS
        object_num = curr_objects;
        Pconst = 1;
        treshold = 0.1; %max cost is 1

        % and use them to normalize
        Pconst = Pconst*(1/maxnorm);

        % compute a cost table between the two images
        costtable = ones(length(image_objects(i)),length(image_objects2(i)))*(treshold + 1);    
        for n = 1:length(image_objects(i).object)
            for m = 1:length(image_objects2(i).object)
                % proximity cost is distance:
                costtable(n,m) = Pconst * cost_proximity(image_objects(i).object(n), image_objects2(i).object(m));
            end
        end
        % get correspondence of the second camera to the first camera
        [index_object] = greedy(costtable, length(image_objects(i).object), length(image_objects2(i).object), treshold);

        % add objects of camera two that are non existant in camera one
        for m = 1:length(image_objects2(i).object)
            % if there wasn't a match, it's an object only on camera 2
            if index_object(m) < 0
                image_objects(i).object(object_num) = image_objects2(i).object(m);
                image_pcs(i).object{object_num} = image_pcs2(i).object{m};
                object_num = object_num + 1;
            % if there was, join both informations
            else
                % merge with 5cm precision
                pc_merged = pcmerge(image_pcs(i).object{index_object(m)}, image_pcs2(i).object{m}, 0.05);
                image_pcs(i).object{object_num} = pc_merged;
                [xmin, xmax, ymin, ymax, zmin, zmax]=getboundingbox(pc_merged);
                % change the box, the frame is the same
                image_objects(i).object(object_num).X = [xmin, xmin, xmin, xmin, xmax, xmax, xmax, xmax];
                image_objects(i).object(object_num).Y = [ymax, ymax, ymin, ymin, ymax, ymax, ymin, ymin];
                image_objects(i).object(object_num).Z = [zmax, zmin, zmin, zmax, zmax, zmin, zmin, zmax];
            end
        end

    end

end
