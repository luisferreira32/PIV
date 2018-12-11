function [maxvol, maxnorm]=get_maxes(film_length, imgsd, cam_params)
% create the vars
maxvol = 0;
maxnorm = 0;

for i = 1:film_length
    % create a point cloud, to get x,y in meters
    depth_array = imgsd(:,:,i);
    Z=double(depth_array(:)');
    [v, u]=ind2sub([480 640],(1:480*640));
    P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
    
    % extract maximum and minimums
    posmin = min(P');
    posmax = max(P');
    
    % create new vol and norm
    newvol = abs(posmax(1)-posmin(1))*abs(posmax(2)-posmin(2))*abs(posmax(3)-posmin(3));
    newnorm = norm([posmax(1), posmax(2), posmax(3)]- [posmin(1), posmin(2), posmin(3)]);
    
    if maxvol < newvol
        maxvol = newvol;
    end
    if maxnorm < newnorm
        maxnorm = newnorm;
    end
end

end