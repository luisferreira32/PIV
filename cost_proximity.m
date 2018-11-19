function [cost] = cost_proximity(object1, object2, A)
    % a cost of proximity is DISTANCE , in this case we are using the
    % objects taken from PIXELS and not METERS, so it's distance in PIXELS
    % which is not optimal.
    cost = 0;
    
    % change point format to vectors and calculate cost
    points1 = zeros(8,3);
    points2 = zeros(8,3);
    for i = 1:8
        points1(i,:) = [object1.X(i), object1.Y(i), object1.Z(i)];
        points2(i,:) = [object2.X(i), object2.Y(i), object2.Z(i)];
        cost = cost + A(i)*norm(points1(i,:) - points2(i,:));
    end
   
end