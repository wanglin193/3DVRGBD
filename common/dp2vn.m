function [pt,nv] = dp2vn( dp, Kdp,factor )

% x/y/z 3-channel image
pt3map = dp2pntmap(dp,Kdp,factor);

% normal map 3-channel image
nvmap = normfrompt3map(pt3map);
% figure,imshow(single(nvmap),[])

% mask out invalid depth
vmask = pt3map(:,:,3)>0;
vmask = vmask & abs(nvmap(:,:,3))>0;
vflag = find(vmask(:));

px = pt3map(:,:,1);  py = pt3map(:,:,2);   pz = pt3map(:,:,3);
pt = [px(:),py(:),pz(:)];
pt = pt(vflag,:);

nx = nvmap(:,:,1); ny = nvmap(:,:,2);  nz = nvmap(:,:,3);
nv = [nx(:),ny(:),nz(:)];
nv = nv(vflag,:);

end