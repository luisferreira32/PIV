function [cost] = cost_proximity(object1, object2)
    % a cost of proximity is DISTANCE , ideally this boxes are from the
    % point clouds so a pretty good measure for the centroid. Other way is
    % to calculate the centroid of the pc itself, but would take more
    % computational time.. (to try in the future)
  
    % objects centroids
    c1 = [mean(object1.X), mean(object1.Y), mean(object1.Z)];
    c2 = [mean(object2.X), mean(object2.Y), mean(object2.Z)];
    
    cost = norm(c1-c2);
   
end