%Exemplo para uma so imagem
imdiff=abs(imgsd(:,:,10)-bgdepth)>.20;
    imgdiffiltered=imopen(imdiff,strel('disk',6));
    figure(3);
    imagesc([imdiff imgdiffiltered]);
    title('Difference image and morph filtered');
    colormap(gray); 
    %figure(4)
    %imshow(imread(d(i).name))
    [L, num]=bwlabel(imgdiffiltered);    % objects label and number of objects
    
    imagesc(bwlabel(imgdiffiltered));
%%
g=regionprops(L,'Area','BoundingBox','Centroid','PixelList');
area=[g.Area];
index= find( area > 500); %Devemos escolher este area como sendo minima possivel para se considerar um objeto??

%objecto especifico
s6=zeros(480,640);
for i=1:640
    for  j=1:480
        if L(j,i)== 2
        s6(:,i)= 1;
        end
    end
end

%% Points
points(1,:)=round([g(index).BoundingBox(1) g(index).BoundingBox(2)]); % deve-se arredondar
points(2,:)=round([g(index).BoundingBox(1) g(index).BoundingBox(2)+g(index).BoundingBox(3)]);
width=g(index).BoundingBox(3);
points(3,:)=round([g(index).BoundingBox(1)+g(index).BoundingBox(4) g(index).BoundingBox(2) ]);
points(3,:)=round([g(index).BoundingBox(1)+g(index).BoundingBox(4) g(index).BoundingBox(2) ]);
points(4,:)=round([g(index).BoundingBox(1)+ g(index).BoundingBox(4) g(index).BoundingBox(2)+g(index).BoundingBox(3) ]);
height=g(index).BoundingBox(4);

%% Bouding box
xmin=g(index).BoundingBox(1);
ymin=g(index).BoundingBox(2);
width=g(index).BoundingBox(3);
height=g(index).BoundingBox(4);
%% Show image selected
figure(4)
imagesc(uint8(reshape(imagewdepth.im(:,:,:,7),[480,640,3])))

%% Crop reconstructed image to include only one object

cropped_object(:,:,1,7)=imcrop(imagewdepth.im(:,:,1,7),[xmin ymin width height]);
cropped_object(:,:,2,7)=imcrop(imagewdepth.im(:,:,2,7),[xmin ymin width height]);
cropped_object(:,:,3,7)=imcrop(imagewdepth.im(:,:,3,7),[xmin ymin width height]);
%%
figure(5)
imshow(uint8(reshape(cropped_object(:,:,:,7),[394,152,3])))


%% Crop depth image 

imagesc(imgsd(:,:,10))
cropped_depth(:,:,10)=imcrop(imgsd(:,:,10),[xmin ymin width height]);
figure(2)
imagesc(reshape(cropped_depth(:,:,10),[394,152]))
a=cropped_depth(:,:,10);

%% Compute point cloud of the object 
Z_crop=double(a(:)')/1000;
[v_c, u_c]=ind2sub([394 152],(1:152*394));
P_c=inv(cam_params.Kdepth)*[Z_crop.*u_c ;Z_crop.*v_c;Z_crop];
pc_c=pointCloud(P_c','color',uint8(reshape(cropped_object(:,:,:,7),[394*152,3])));
showPointCloud(pc_c)
a=boundary(pc_c);

%% Find indexes for an object 
idexes=find(