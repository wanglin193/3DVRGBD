function nmap = normfrompt3map(pt3map)
 
[h,w,c]=size(pt3map);
if(c==1)
    error('input map must be x/y/z 3layer');
end

nmap = zeros(h,w,3,'single');
%nmap(:,:,3)=-1;

immask = pt3map(:,:,3)>0;
immask(1,:)=0;
immask(end,:)=0;
immask(:,1)=0;
immask(:,end)=0;

SE =[0,1,0;1,1,1;0,1,0];
mask_thin = imerode(immask,SE);
imcont = immask-mask_thin;
%figure,imshow(imcont+immask,[])
% dx = imdilate(pt3map(:,:,1),SE);  dy = imdilate(pt3map(:,:,2),SE); dz = imdilate(pt3map(:,:,3),SE);
% pt3map(:,:,1) = imgaussfilt(dx,2);  pt3map(:,:,2) = imgaussfilt(dy,2); pt3map(:,:,3) = imgaussfilt(dz,2); 

%计算四邻叉乘
p0 = pt3map(2:h-1,2:w-1,:);
p{1} = pt3map(2:h-1,3:w,:);
p{2} = pt3map(3:h,2:w-1,:);
p{3} = pt3map(2:h-1,1:w-2,:);
p{4} = pt3map(1:h-2,2:w-1,:);

v2 = reshape(p{1}-p0,(h-2)*(w-2),3);
v3 = reshape(p{2}-p0,(h-2)*(w-2),3);
v1 = cross(v3,v2);

v2 = reshape(p{2}-p0,(h-2)*(w-2),3);
v3 = reshape(p{3}-p0,(h-2)*(w-2),3);
v1 = v1+ cross(v3,v2);

v2 = reshape(p{3}-p0,(h-2)*(w-2),3);
v3 = reshape(p{4}-p0,(h-2)*(w-2),3);
v1 = v1+ cross(v3,v2);

v2 = reshape(p{4}-p0,(h-2)*(w-2),3);
v3 = reshape(p{1}-p0,(h-2)*(w-2),3);
v1 = v1+ cross(v3,v2);

n1 = sqrt(sum(v1.*v1,2));
%idx = n1<0.001 ;
%v1(idx,:)=0*v1(idx,:);
%n1(idx)=1;
v1 = v1./repmat(n1,1,3); 
%idx = n1<0.000001 ;
%v1(idx,:)=0*v1(idx,:);

%idx1 = abs(v1(:,1))>0.95 | abs(v1(:,2))>0.95 | abs(v1(:,3))<0.00001;
%v1(idx1,:)=0*v1(idx1,:);

%contour pixel has norm [0,0,1]
%v1(idx,3)=1;
nmap(2:h-1,2:w-1,:) = reshape(v1,(h-2),(w-2),3);
for i=1:3
    nmap(:,:,i)=nmap(:,:,i).*mask_thin;
end
%figure,imshow(nmap,[]);
%% 处理边缘点
idx = find(imcont(:)>0);
%[i,j] = ind2sub([h,w],idx);
nx = nmap(:,:,1);
ny = nmap(:,:,2);
nz = nmap(:,:,3);
nx1 = nx(idx+h) + nx(idx-h) + nx(idx+1) + nx(idx-1) ;
ny1 = ny(idx+h) + ny(idx-h) + ny(idx+1) + ny(idx-1) ;
nz1 = nz(idx+h) + nz(idx-h) + nz(idx+1) + nz(idx-1) ;
nn = sqrt(nx1.^2+ny1.^2+nz1.^2);
idx1 = nn<0.0001;
nx1 = nx1./(nn+0.0001); nx1(idx1)=0;
ny1 = ny1./(nn+0.0001);ny1(idx1)=0;
nz1 = nz1./(nn+0.0001);nz1(idx1)=0;
%nn=reshape(n1,(h-1),(w-1));
nx(idx)=nx1;
ny(idx)=ny1;
nz(idx)=nz1;
nmap(:,:,1) = reshape(nx,h,w);
nmap(:,:,2) = reshape(ny,h,w);
nmap(:,:,3) = reshape(nz,h,w);
%figure,imshow(nmap,[]);
return
 


    
