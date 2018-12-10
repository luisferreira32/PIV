function [cost] = cost_colour(pc1, pc2)
    % a cost of colour is histogram diference! let's try it
  
    % swap from rgb to hsv
    % this is a cost with the colours having two object rgb pixels.
    hsv1 = rgb2hsv(double(pc1.Color)./255);
    hsv2 = rgb2hsv(double(pc2.Color)./255);
 
    % if saturation is too little we must ignore hue and do with saturation
    linind1 = find(hsv1(:,2)<.05);
    linind2 = find(hsv2(:,2)<.05);
    % the 50000 was random ~16%, img of size 480*640 =~ 300k
    if length(linind1) > 50000 || length(linind2) > 50000
        h1 = imhist(hsv1(:,2),256);
        h2 = imhist(hsv2(:,2),256);
        %normalize
        h1=h1/size(hsv1,1)/size(hsv1,2);
        h2=h2/size(hsv2,1)/size(hsv2,2);
    else
        % maybe nullify the lower saturation hues?
        h1 = imhist(hsv1(:,1),256);
        h2 = imhist(hsv2(:,1),256);
        %normalize
        h1=h1/size(hsv1,1)/size(hsv1,2);
        h2=h2/size(hsv2,1)/size(hsv2,2);
    end

    % returns a cost between 0 and 1?
    cost = pdist2(h1',h2');
end
