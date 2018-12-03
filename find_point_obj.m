%Exemplo para uma so imagem
imdiff=abs(imgsd(:,:,11)-bgdepth)>.20;
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

nb(:,:,1,10)=imagewdepth.im(:,:,1,10).*s6;
nb(:,:,2,10)=imagewdepth.im(:,:,2,10).*s6;
nb(:,:,3,10)=imagewdepth.im(:,:,3,10).*s6;
pc=pointCloud(P', 'color',uint8(reshape(nb(:,:,:,10),[480*640,3])));
figure(5);showPointCloud(pc);
%%
%%Points
points(1,:)=round([g(index).BoundingBox(1) g(index).BoundingBox(2)]); % deve-se arredondar
points(2,:)=round([g(index).BoundingBox(1) g(index).BoundingBox(2)+g(index).BoundingBox(3)]);

points(3,:)=round([g(index).BoundingBox(1)+g(index).BoundingBox(4) g(index).BoundingBox(2) ]);
points(3,:)=round([g(index).BoundingBox(1)+g(index).BoundingBox(4) g(index).BoundingBox(2) ]);
points(4,:)=round([g(index).BoundingBox(1)+ g(index).BoundingBox(4) g(index).BoundingBox(2)+g(index).BoundingBox(3) ]);




        

%%

