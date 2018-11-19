function [objects] = get_path(size, image_objects, imglabel,imgnum, imgsrt)
% to get the path we must choose the minimum (acceptable) cost between two
% objects in two diferent pictures. cost function f = A*proximity + B*color

% change this scalers to get better cost
% proximity of vertices
A = [1,1,1,1,1,1,1,1];
B = 1;
threshold = 500;


% declare cost tables struct
costs(size) = struct();

% for each image calculate cost table
for i=1:(size-1)
    % start all with an impossible cost (undefined)
    costs(i).table = ones(imgnum(i),imgnum(i+1))*(-1);
    
    % for each pair define a cost
    for n = 1:imgnum(i)
        for m = 1:imgnum(i+1)
            % distance, note that if we use imgwdepth we could convert back
            % to meters and get a better understanding of distances, here
            % it's simplified as pixels distance
            costs(i).table(n,m) = cost_proximity(image_objects(i).object(n), image_objects(i+1).object(m), A);
            
            % the colour cost should be done with hue and/or saturation
            costs(i).table(n,m) = costs(i).table(n,m); % + cost_color(imgrt(i), imgrt(i+1), imglabel(i), imglabel(i+1), B)
        end
    end
    
end


end
