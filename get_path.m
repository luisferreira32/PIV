function [objects] = get_path(film_length, image_objects, image_pcs, imgnum)
% to get the path we must choose the minimum (acceptable) cost between two
% objects in two diferent pictures. cost function f = A*proximity + B*color

% change this scalers to get better cost
% proximity of vertices
Pconst = 1;
Vconst = 1;
Cconst = 1;
threshold = 500;


% declare cost tables struct
costs(film_length) = struct();

% for each image calculate cost table
for i=1:(film_length-1)
    % start all with an impossible cost (undefined)
    costs(i).table = ones(imgnum(i),imgnum(i+1))*(-1);
    
    % for each pair define a cost
    for n = 1:imgnum(i)
        for m = 1:imgnum(i+1)
            % proximity cost is distance:
            costs(i).table(n,m) = Pconst * cost_proximity(image_objects(i).object(n), image_objects(i+1).object(m));
            % we can have a volume cost
            cost(i).table(n,m) = cost(i).table(n,m); % + Vconst * cost_volume(pc1, pc2);
            
            % the colour cost should be done with hue and/or saturation
            costs(i).table(n,m) = costs(i).table(n,m); % + Cconst * cost_color(image_pcs(i).object(n), image_pcs(i+1).object(m));
        end
    end
    
end

%% Assign with greedy algorithm but counting on threshhold


end
