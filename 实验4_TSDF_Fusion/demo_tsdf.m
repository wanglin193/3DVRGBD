%TSDF:Truncated Signed Distance Function
clear all; close all;
wid=64; hei=64; thick=64;
center = [hei,wid,thick]/2;
rad = 20; len = 30;

[x,y,z] = meshgrid( 1:wid,1:hei,1:thick );
x=single(x); y=single(y); z=single(z);

%球体
v1 = (x-center(1)).^2+(y-center(2)).^2+(z-center(3)).^2;
v1 = sqrt(v1) - rad;
%正方体
v2 = max(abs(x-center(1)), abs(y-center(2)));
v2 = max(v2, abs(z-center(3))) - len/2;

th_truncate = 8;
v1( v1>th_truncate | v1<-th_truncate ) = NaN; v1 = v1/th_truncate;
v2( v2>th_truncate | v2<-th_truncate ) = NaN; v2 = v2/th_truncate;

[Face1,Verts1] = isosurface(v1,0);
[Face2,Verts2] = isosurface(v2,0);


figure ;
ax2 = subplot(1,2,1);
slice(v1,wid/2,hei/2,thick/2);colormap(ax2,jet(16));axis equal
title('TSDF slice: sphere')
ax2 = subplot(1,2,2);
slice(v2,wid/2,hei/2,thick/2);colormap(ax2,jet(16));axis equal
title('TSDF slice: cube')

fig = figure ;
subplot(1,2,1) 
p = patch('Faces',int32(Face1),'Vertices',Verts1);
%isonormals(volume,p)
p.FaceColor = [0.8,0.8,0.7];p.EdgeColor = 'none';
daspect([1,1,1])
xlim([1,wid]);ylim([1,hei]);zlim([1,thick]);
view(3); camlight;%lighting gouraud

subplot(1,2,2)
p = patch('Faces',int32(Face2),'Vertices',Verts2);
p.FaceColor = [0.8,0.8,0.7];p.EdgeColor = 'none';
daspect([1,1,1])
xlim([1,wid]);ylim([1,hei]);zlim([1,thick]);
view(3); camlight;%lighting gouraud


