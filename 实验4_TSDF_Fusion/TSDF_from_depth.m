function [SDF,Wt] = TSDF_from_depth(dp,pose,Kdp,vol_res,voxelsize,offset_model_vol,range)
%   Truncated distance volume of a depth image
%   [SDF,Wt] = TSDF_from_depth( dp,pose,Kdp,vol_res,voxelsize,offset_model_vol,range );
%   dp - 2D-depth, 
%   pose - RT of volume in camera coordinates
%   Kdp - Intrisinc of camera, 3*3 matrix
%   vol_res   - size of voxels, for example  [256,512,256] 
%   voxelsize - size of a voxel(mm), cube size [voxelsize,voxelsize,voxelsize]
%   offset_model_vol - volume original in model space , such as [-700;-700;-800]
%   range - range of depth, for example [100,2000] mean 100mm to 2m                    
%   SDF, Weight - Volumn of distmap, and weights {0,1}
%  by Wang Lin

dp(dp<range(1) | dp>range(2)) = NaN;
 
% coordinates of voxel grids
[x,y,z] = meshgrid( 1:vol_res(1), 1:vol_res(2), 1:vol_res(3) );
x=single(x*voxelsize)+offset_model_vol(1);
y=single(y*voxelsize)+offset_model_vol(2); 
z=single(z*voxelsize)+offset_model_vol(3);

% SDF weight
Wt = ones(vol_res(2),vol_res(1),vol_res(3),'uint8');

% [R|T] * [x;y;z], volume to camera space              
p3x = single(pose(1,1)*x + pose(1,2)*y + pose(1,3)*z + pose(1,4));
p3y = single(pose(2,1)*x + pose(2,2)*y + pose(2,3)*z + pose(2,4));
p3z = single(pose(3,1)*x + pose(3,2)*y + pose(3,3)*z + pose(3,4));

% p2d = K*P3d, project points onto depth map.
p2dx = Kdp(1,1)*(p3x./p3z) + Kdp(1,3);
p2dy = Kdp(2,2)*(p3y./p3z) + Kdp(2,3);

% sdf = v2d - z
v2d_depth = interp2( dp, p2dx, p2dy, 'nearest', NaN ); %越界置NaN
SDF = single( v2d_depth - p3z );

% Truancated SDF
Th_trunc = 4*voxelsize; %尽可能小
msk = SDF>Th_trunc | SDF<-Th_trunc;
SDF(msk) = NaN; Wt(msk) = 0;
SDF = SDF/Th_trunc;
return
 