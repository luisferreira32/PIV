function [R21, T21] = get_RT21(film_length, imgsrt1, imgsrt2, imgd1, imgd2, cam_params)
    %% Images selected
    samples=fix(0.10*film_length);
    img_index=fix(rand(samples,1)*film_length)+1;
    
    for i=1:length(img_index)
        dp1(:,:,i)=imgd1(:,:,img_index(i));
        dp2(:,:,i)=imgd2(:,:,img_index(i));
        rgbd1(:,:,:,i)=imgsrt1(:,:,:,i);
        rgbd2(:,:,:,i)=imgsrt2(:,:,:,i);        
    end
    
	%% Compute 3d in camera frame
    [v, u]=ind2sub([480 640],(1:480*640));
    for i=1:length(img_index)
        % cam 1
        Z = dp1(:,:,i);
        Z = Z(:)';
        P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
        xyz1(:,:,i) = P';
        % cam 2
        Z = dp2(:,:,i);
        Z = Z(:)';
        P=inv(cam_params.Kdepth)*[Z.*u ;Z.*v;Z];
        xyz2(:,:,i) = P';
    end
    %% Perform SIFT for all images 
    P1=[];
    P2=[];

    for i=1:length(img_index)
        I1=single(rgb2gray(uint8(rgbd1(:,:,:,i))));
        I2=single(rgb2gray(uint8(rgbd2(:,:,:,i))));
        [f1,d1_]=vl_sift(I1);
        [f2,d2_]=vl_sift(I2);
        [matches, ~] = vl_ubcmatch(d1_, d2_, 3);
        y1=round(f1(2,matches(1,:)));
        x1=round(f1(1,matches(1,:)));
        y2=round(f2(2,matches(2,:)));
        x2=round(f2(1,matches(2,:)));
        ind1=sub2ind(size(dp2(:,:,i)),y1,x1);
        ind2=sub2ind(size(dp2(:,:,i)),y2,x2);
        p1=xyz1(ind1,:,i);
        p2=xyz2(ind2,:,i);
        P1=[P1; p1];
        P2=[P2; p2];
    end
    
    %% Ransac
    n_it=1000;
    threshold=0.1;
    num_in=[];
    trans=[];
    % Generate random numbers
    n_points=fix(rand(4*n_it,1)*length(P1))+1;

    for i=1:n_it-3
        % Choose random points, we need at least 4 points for the procrustes
        rand_points=n_points(4*i:4*i+3);
        % Get 3D points correspondent to theses indexes

        X1=P1(rand_points,:);
        X2=P2(rand_points,:);

        % Compute procrustes
        [~, ~, transform]=procrustes(X1,X2, 'scaling', false, 'reflection', false);

        % Compute points from image 2 projected to image 1 using the
        % transformation from procrustes

        xyz21= P2 * transform.T + repmat(transform.c(1,:),length(P2),1);
        error=sqrt(sum((P1-xyz21).^2,2)); % Compute the error for all points

        inds(i).in=find(error<threshold);

        % Add the number of inliers to the inliers vector

        num_in=[num_in length(inds(i).in)];
        % Save the transformation in each iteration
        trans=[trans transform];
    end
    
    [~, ind]=max(num_in);
    xyz1f=P1(inds(ind).in,:);
    xyz2f=P2(inds(ind).in,:);
    [~, ~, transform]=procrustes(xyz1f, xyz2f, 'scaling', false, 'reflection', false);
    R21=transform.T;
    T21=transform.c(1,:);

end
