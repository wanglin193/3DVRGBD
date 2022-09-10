function [vertex,face,vmask] = dp2mesh( dp, fol,factor )
%  dp2mesh: Construct mesh from depth map ,with focal length, remove
%           triangles with long edges automatically.
%
%   [vertex,face,vmask] = dp2mesh( dp, fol,factor )
%   or, with K
%   [vertex,face,vmask] = dp2mesh( dp, K,factor )
%       dp - depth map
%       fol/K - folcal length (pixel) or intrinsic matrix
%       factor - sampling interval, such as 1,2,4,8 ... 
%
%   For example:
%
%       dp = dpread('test.dpt');
%       msk = dp>400 & dp<3000;
%       dp = dp.*msk;
%       [vertex,face,vmask] = dp2mesh( dp,571,2 );
%       trimesh(face,vertex(:,1),vertex(:,2),vertex(:,3));
%       view(0,-90);
%       axis equal
%
%   See also : dp2pnt, dp2pt3map

if( nargin<2)
    error('Need K or focal_length.');
end

if( nargin<3)
    factor=1;
end

[h1,w1]=size(dp);
if( size(fol,1)==1 && size(fol,2)==1 )
    u0 = w1/2;
    v0 = h1/2;
elseif( size(fol,1)==3 && size(fol,2)==3 )
    K = fol;
    fol= K(1,1);
    u0 = K(1,3);
    v0 = K(2,3);
else
    error('K or focal_length error.')
end

X2dmap = repmat((1:w1) -u0,h1,1);
Y2dmap = repmat((1:h1)'-v0,1,w1);
X2dmap = X2dmap(factor:factor:h1,factor:factor:w1);
Y2dmap = Y2dmap(factor:factor:h1,factor:factor:w1);
dp = dp(factor:factor:h1,factor:factor:w1);

x3d = dp.*X2dmap/fol;
y3d = dp.*Y2dmap/fol;

vertex = single([x3d(:),y3d(:),dp(:)]) ;
vmask = dp>0;
vflag = vmask(:);

%nonzero z vertex
vnum = sum(vflag);
vid =zeros(size(vflag));
%set nonzero index as 1,2,3,....
vid(vflag)=[1:vnum];
%remove nonzero vertexo
vertex = vertex(vflag,:);

%% triangles
[h,w]=size(dp);
% indice of pnt
[xd,yd]=meshgrid(2:w,2:h);
idx = sub2ind([h,w],yd(:),xd(:));
face = [idx, idx-h-1,idx-h;idx, idx-1,idx-h-1];

[null,id] = sort(face(:,1));
face = face(id,:);
%
fid=[vid(face(:,1)),vid(face(:,2)),vid(face(:,3))];
fflag = (fid(:,1) & fid(:,2) & fid(:,3));
face = fid(fflag,:);

%% check edge length
v1=vertex(face(:,1),:);
v2=vertex(face(:,2),:);
v3=vertex(face(:,3),:);
d1=sqrt(sum((v1-v2).*(v1-v2),2));
d2=sqrt(sum((v1-v3).*(v1-v3),2));
d3=sqrt(sum((v2-v3).*(v2-v3),2));
th = (16/factor)*3*(mean(d1)+mean(d2)+mean(d3))/3;
%th = 30;
face = face( d1<th & d2<th &d3<th , : );

return


  