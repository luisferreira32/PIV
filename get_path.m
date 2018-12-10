function [objects] = get_path(film_length, image_objects, imgsrt, imgsd)
% to get the path we must choose the minimum (acceptable) cost between two
% objects in two diferent pictures. cost function f = A*proximity + B*color

% change this scalers to get better cost
Pconst = 1;
Vconst = 1;
Cconst = 1;
treshold = 1; % all costs are normalized, sum is 3

% use proximity constant to normalize (vector from one edge to the other)
maxnorm = norm([480, 640, max(max(max(imgsd)))]-[0,0,min(min(min(imgsd)))]);
Pconst = Pconst*(1/maxnorm);
% have max volume to normalize volumes
maxvol = 480*640*(max(max(max(imgsd)))-min(min(min(imgsd))));
Vconst = Vconst*(1/maxvol);

% allocate memory before
costs(film_length) = struct();
objects(length(image_objects(1).object)) = struct();

% to keep tracking of the objects
object_index = zeros(1,length(image_objects(1).object));
total_objs = 0;

% first "wave" of objects, no matching yet
for i = 1:length(image_objects(1).object)
    object_index(i) = total_objs+1;
    total_objs = total_objs+1;
    objects(object_index(i)).X = [image_objects(1).object(object_index(i)).X];
    objects(object_index(i)).Y = [image_objects(1).object(object_index(i)).Y];
    objects(object_index(i)).Z = [image_objects(1).object(object_index(i)).Z];
    objects(object_index(i)).frames_tracked = [image_objects(1).object(object_index(i)).frames_tracked];
end

% for each image calculate cost table
for i=1:(film_length-1)
    % start all with an impossible cost
    costs(i).table = ones(length(image_objects(i)),length(image_objects(i+1)))*(treshold+1);
    
    % for each pair define a cost
    for n = 1:length(image_objects(i).object)
        for m = 1:length(image_objects(i+1).object)
            % proximity cost is distance:
            costs(i).table(n,m) = Pconst * cost_proximity(image_objects(i).object(n), image_objects(i+1).object(m));
            % we can have a volume cost
            costs(i).table(n,m) = costs(i).table(n,m) + Vconst * cost_volume(image_objects(i).object(n), image_objects(i+1).object(m));
            
            % the colour cost should be done with hue and/or saturation
            costs(i).table(n,m) = costs(i).table(n,m) + Cconst * cost_colour(image_objects(i).object(n).pixel_list, image_objects(i+1).object(m).pixel_list, imgsrt(:,:,:,i));
            
        end
    end
      
    % Assign with greedy algorithm
    [index_object] = greedy(costs(i).table, length(image_objects(i).object),length(image_objects(i+1).object), treshold);
        
    % and make the final object struct
	object_index_aux = zeros(1,length(image_objects(i+1).object));
    for m = 1:length(image_objects(i+1).object)
    	% if there was a match
		if index_object(m) > 0
			% track object
            objects(object_index(index_object(m))).X = [objects(object_index(index_object(m))).X;image_objects(i+1).object(m).X];
            objects(object_index(index_object(m))).Y = [objects(object_index(index_object(m))).Y;image_objects(i+1).object(m).Y];
            objects(object_index(index_object(m))).Z = [objects(object_index(index_object(m))).Z;image_objects(i+1).object(m).Z];
            objects(object_index(index_object(m))).frames_tracked = [objects(object_index(index_object(m))).frames_tracked, image_objects(i+1).object(m).frames_tracked];
            % register for next iteration
            object_index_aux(m) = object_index(index_object(m));
		else
			% create new object
    		total_objs = total_objs+1;
            objects(total_objs).X = [image_objects(i+1).object(m).X];
            objects(total_objs).Y = [image_objects(i+1).object(m).Y];
            objects(total_objs).Z = [image_objects(i+1).object(m).Z];
            objects(total_objs).frames_tracked = [image_objects(i+1).object(m).frames_tracked];
            % register for next iteration
			object_index_aux(m) = total_objs;
		end 
    end
    % this is the list of objects existant in the next frame
    object_index = object_index_aux;

end


end
