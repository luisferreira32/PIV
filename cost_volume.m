function [cost] = cost_volume(object1, object2)
    % a cost of proximity is DISTANCE , ideally this boxes are from the
    % point clouds so a pretty good measure for the centroid. Other way is
    % to calculate the centroid of the pc itself, but would take more
    % computational time.. (to try in the future)
  
    % objects centroids
    V1 = (max(object1.X)-min(object1.X))*(max(object1.Y)-min(object1.Y))*(max(object1.Z)-min(object1.Z));
    V2 = (max(object2.X)-min(object2.X))*(max(object2.Y)-min(object2.Y))*(max(object2.Z)-min(object2.Z));
    cost = abs(V1-V2);
   
end