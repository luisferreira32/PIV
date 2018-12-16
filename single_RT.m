% CORRECT THIS FUNCTION, GIVEN TWO RGBD, TWO DEPTHS, AND CAM PARAMS

function [R21, T21] = single_RT(rgbd1, rgbd2, imgsd1, imgsd2, cam_params)
	% get 3d points
	[v, u]=ind2sub([480 640],(1:480*640));
	Z = double(imgsd1(:)');
    xyz1=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
	Z = double(imgsd2(:)');
    xyz2=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];

	% Choose image to get matching points - SIFT
	I1=single(rgb2gray(rgbd1));
	I2=single(rgb2gray(rgbd2));
	[f1,d1]=vl_sift(I1);
	[f2,d2]=vl_sift(I2);
	[matches, scores] = vl_ubcmatch(d1, d2);
	%% Get the index of the matches in the image 
	y1=round(f1(2,matches(1,:)));
	x1=round(f1(1,matches(1,:)));
	y2=round(f2(2,matches(2,:)));
	x2=round(f2(1,matches(2,:)));
	ind1=sub2ind(size(imgsd2),y1,x1);
	ind2=sub2ind(size(imgsd2),y2,x2);

	%% Compute the 3D coordinates of the matches
	P1=xyz1(ind1,:);
	P2=xyz2(ind2,:);

	%% Ransac
	n_it=2000; % ??
	threshold=0.005;
	num_in=[];
	trans=[];
	% Generate random numbers
	n_points=fix(rand(4*n_it,1)*length(matches))+1;

	espera=1;

	for i=1:n_it-3
	    % Choose random points, we need at least 4 points for the procrustes
	    rand_points=matches(:,n_points(4*i:4*i+3));
	    ind1=sub2ind(size(imgsd2),round(f1(2,rand_points(1,:))),round(f1(1,rand_points(1,:))));
	    ind2=sub2ind(size(imgsd2),round(f2(2,rand_points(2,:))),round(f2(1,rand_points(2,:))));
	    
	    % Get 3D points correspondent to theses indexes
	    X1=xyz1(ind1,:);
	    X2=xyz2(ind2,:);
	    
	    % Compute procrustes
	    [d, Z, transform]=procrustes(X1,X2);
	    
	    % Compute points from image 2 projected to image 1 using the
	    % transformation from procrustes
	    
	    xyz21= transform.b * P2 * transform.T + repmat(transform.c(1,:),length(P2),1);
	    error=sqrt(sum((P1-xyz21).^2,2)); % Compute the error for all points
	    
	    inds(i).in=find(error<2);
	    
	    % Add the number of inliers to the inliers vector
	    
	    num_in=[num_in length(inds(i).in)];
	    % Save the transformation in each iteration
	    trans=[trans transform];
	   
	    % Waitbar
	    waitbar(espera/(n_it-3))
	    espera=espera+1;
	    
	end

	%% Find the best result 

	[m, ind]=max(num_in);
	xyz1f=P1(inds(ind).in,:);
	xyz2f=P2(inds(ind).in,:);
	[d, T, transform]=procrustes(xyz1f, xyz2f);
	R=transform.T;
	T=transform.c(1,:);

end