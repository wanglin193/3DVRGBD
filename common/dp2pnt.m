function [p3d,msk] = dp2pnt( dpmap, fol ,factor )
%   dp2pnt: 3d points from depth map with K or focal length
%
%   [p3d,msk] = dp2pnt( dpmap, fol ,factor )
%       dpmap - depth map
%       K/fol - intrisinc of depth camera K [3,3], fol(pixel) focal length 
%       factor - sampling interval, such as 1,2,4,8 ... 
%    p3d - 3*n 3d points matrix.
%
%   [p3d,vmsk] = dp2pnt( dpmap, fol ,factor )
%   % or
%   [p3d,vmsk] = dp2pnt( dpmap, K ,factor )
%   % For factor = 1:
%   [p3d,vmsk] = dp2pnt( dpmap, fol )
%
%	For example :
%       dp = dpread('test.dpt');
%       msk = dp>400 & dp<3000;
%       dp = dp.*msk;
%       pt = dp2pnt(dp,571,4);
%       plot3(pt(1,:),pt(2,:),pt(3,:),'.');
%       view(0,-90);
%       axis equal
%
%       See also : dp2mesh, dp2pt3map

if( nargin<1)
    help dp2pnt
    return
end

if( nargin<2)
    error('Need K or focal_length.');
end

if( nargin<3)
    factor=1;
end
 
[h,w]=size(dpmap);
if( size(fol,1)==1 && size(fol,2)==1 )
    u0 = w/2;
    v0 = h/2;
elseif( size(fol,1)==3 && size(fol,2)==3 )
    K = fol;
    fol = K(1,1);
    u0 = K(1,3);
    v0 = K(2,3);
else   
    error('K or focal_length error.')
end

X2dmap = repmat((1:w)-u0,h,1);
Y2dmap = repmat((1:h)'-v0,1,w);

X2dmap =X2dmap(factor:factor:h,factor:factor:w);
Y2dmap =Y2dmap(factor:factor:h,factor:factor:w);
dpmap = dpmap (factor:factor:h,factor:factor:w);

Z0  = single(dpmap(:));
X2d = X2dmap(:);
Y2d = Y2dmap(:);

msk = Z0>0;

X0 = Z0(msk).*X2d(msk)/fol;
Y0 = Z0(msk).*Y2d(msk)/fol;

p3d = single([X0 ,Y0, Z0(msk)]');

return
  