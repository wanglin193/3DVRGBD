function p3dmap = dp2pntmap( dpmap, fol ,factor )
%
% p3dmap = dp2pntmap( dpmap, fol ,factor )
% p3dmap = dp2pntmap( dpmap, K ,factor )
%output a h*w*3 structured point map,
%containing x,y,z value of 3d point
% 2019.10.17

if( nargin<2)
    %kinect
    fol = 580;
    factor=1;
end
dpmap(dpmap<100)=0;

[h,w]=size(dpmap);

if( size(fol,1)==1 && size(fol,2)==1 )
    [h1,w1]=size(dpmap);
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


X2dmap = repmat((0:w-1)-u0,h,1);
Y2dmap = repmat((0:h-1)'-v0,1,w);

X2dmap =X2dmap(factor:factor:h,factor:factor:w);
Y2dmap =Y2dmap(factor:factor:h,factor:factor:w);
dpmap=dpmap(factor:factor:h,factor:factor:w);

Xmap = dpmap.*X2dmap/fol;
Ymap = dpmap.*Y2dmap/fol;
p3dmap=cat(3,Xmap,Ymap,dpmap);

return
  