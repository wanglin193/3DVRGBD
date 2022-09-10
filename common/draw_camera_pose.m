%  by Wang Lin
%  2022.7.3
function draw_camera_pose(im,Kc,pose,scale)
%% daw camera frames

map = jet(256) ; %gray(256); 
[h,w,ch]=size(im);
if(ch==1)
    im_range = max(im(:))-min(im(:));
    im_range = 256/im_range;
else
    im_range=1;
end

% rectangle picture frame
x = [1,w;1,w]-Kc(1,3);  x=x*scale;
y = [1,1;h,h]-Kc(2,3);  y=y*scale;
z = Kc(1,1)+x*0;        z=z*scale; 
% 4 corners of picture frame
corners=[x(1,1),y(1,1),z(1,1);
x(end,1),y(end,1),z(end,1);
x(1,end),y(1,end),z(1,end);
x(end,end),y(end,end),z(end,end)];
corners = [corners,ones(4,1)];

% to 3D frame
pt=[x(:),y(:),z(:),ones(numel(x),1)]*pose(1:3,:)';
[h_,w_,~]=size(x); 
x=reshape(pt(:,1),h_,w_);
y=reshape(pt(:,2),h_,w_);
z=reshape(pt(:,3),h_,w_);
texture_map(x,y,z,im*im_range,map);

% draw coordinate frames
len = min(w,h)*scale;
frame = [0,0,0,1; 
         len,0,0,1;
         0,len,0,1;
         0,0,len,1;
         corners ]';
fr = pose(1:3,:)*frame;
 
plot3(fr(1,[1,2]),fr(2,[1,2]),fr(3,[1,2]),'r');
plot3(fr(1,[1,3]),fr(2,[1,3]),fr(3,[1,3]),'g');
plot3(fr(1,[1,4]),fr(2,[1,4]),fr(3,[1,4]),'b');
for i=5:8
    plot3(fr(1,[1,i]),fr(2,[1,i]),fr(3,[1,i]),'c');
end
end

%% texture mapping by surface, code from warp()'
function handle = texture_map(x,y,z,cdata,map)
axHandle = newplot;
h = surface(x,y,z,cdata,'EdgeColor','none','FaceColor','texturemap','CDataMapping','direct');
if (~isempty(map))
    axHandle.ColorSpace.Colormap = map;
end
if nargout, handle = h; end
end
