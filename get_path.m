function [objects] = get_path(film_length, image_objects, image_pcs)
% to get the path we must choose the minimum (acceptable) cost between two
% objects in two diferent pictures. cost function f = A*proximity + B*color

% change this scalers to get better cost
% proximity of vertices
Pconst = 1;
Vconst = 1;
Cconst = 1;
treshold = 500;


% allocate memory before
costs(film_length) = struct();
objects(length(image_objects(1))) = struct();
object_num = [];
total_objs = 0;

% first "wave" of objects, no matching yet
for i = 1:length(image_objects(1))
    object_num = [object_num, total_objs+1];
    total_objs = total_objs+1;
    objects(object_num(i)).X = [image_objects(1).object(object_num(i)).X];
    objects(object_num(i)).Y = [image_objects(1).object(object_num(i)).Y];
    objects(object_num(i)).Z = [image_objects(1).object(object_num(i)).Z];
    objects(object_num(i)).frames_tracked = [image_objects(1).object(object_num(i))).frames_tracked];
end

% for each image calculate cost table
for i=1:(film_length-1)
    % start all with an impossible cost
    costs(i).table = ones(length(image_objects(i)),length(image_objects(i+1)))*(treshold+1);
    
    % for each pair define a cost
    for n = 1:length(image_objects(i))
        for m = 1:length(image_objects(i+1))
            % proximity cost is distance:
            costs(i).table(n,m) = Pconst * cost_proximity(image_objects(i).object(n), image_objects(i+1).object(m));
            % we can have a volume cost
            %costs(i).table(n,m) = costs(i).table(n,m); % + Vconst * cost_volume(pc1, pc2);
            
            % the colour cost should be done with hue and/or saturation
            costs(i).table(n,m) = costs(i).table(n,m) + Cconst * cost_colour(image_pcs(i).object{n}, image_pcs(i+1).object{m});
        end
    end
    
    % Assign with greedy algorithm
    match_object = ones(length(image_objects(i)))*(treshold + 1);
    index_object = ones(length(image_objects(i)))*(-1);
    for n = 1:length(image_objects(i))
        for m = 1:length(image_objects(i+1))
            if match_object(n) > costs(i).table(n,m) && costs(i).table(n,m) < treshold
                match_object(n) = costs(i).table(n,m);
                index_object(n) = m;
            end
        end
    end
    for m = 1:length(image_objects(i+1))
        % check which of the columns is the lowest value and make all
        % others impossible
        aux = treshold+1;
        index = find(index_object == m);
        for n = 1:length(index)
            if match_object(index(n)) < aux
                aux = match_object(index(n));
            else
                match_object(index(n)) = treshold +1;
            end
        end
    end
    
    % and make the final object struct
    % IT IS WRONG: CHECK OBJECT_NUM, IF ONE FIGURE AS MORE OBJECTS OR LESS
    % THAN THE OTHER, AND ACT ACCORDINGLY!!
    for n = 1:length(image_objects(i))
        % if there was a match, it's the same object
        if match_object(n) < treshold
            objects(object_num(n)).X = [objects(object_num(n)).X;image_objects(i+1).object(index_object(n)).X];
            objects(object_num(n)).Y = [objects(object_num(n)).Y;image_objects(i+1).object(index_object(n)).Y];
            objects(object_num(n)).Z = [objects(object_num(n)).Z;image_objects(i+1).object(index_object(n)).Z];
            objects(object_num(n)).frames_tracked = [objects(object_num(n)).frames_tracked, image_objects(i+1).object(index_object(n)).frames_tracked];
        % if there wasn't, new object!
        else
            object_num(n) = total_objs + 1;
            total_objs = total_objs + 1;
        end
    end
    
end


end
