function [depth]=gen_depth_from_mesh(V,F,wid,hei,kcam ,Rt)
%kcam = [fol,0,wid/2;0,fol,hei/2;0,0,1];
numV = size(V,1);
Vcam = [V,ones(numV,1)]*Rt';
%figure(2),plot3(Vcam(:,1),Vcam(:,2),Vcam(:,3),'.') ;view(0,-90);axis equal
[nv,nf] = compute_normal(Vcam',F'); nv = nv'; nf=nf';
 
%idx=nv(:,3)<0.0;
%figure(3),plot3(Vcam(idx,1),Vcam(idx,2),Vcam(idx,3),'.') ;view(0,-90);;axis equal
%每个三角形投影到深度图位置，估计占据的像素
V2d = Vcam*kcam';
for i=1:3, V2d(:,i)=V2d(:,i)./V2d(:,3); end
depth = NaN*ones(hei,wid);
v_ = zeros(3,3);
for i=1:size(F,1)
    if nf(i,3)> 0.3  
        continue;
    end 
 
    v_(1:3,:) = Vcam(F(i,1:3),:);
    if( max(v_(1:3,3))<0.2 ) %z very near to camera
        continue;
    end
    
    v_2d = V2d(F(i,1:3),:);
    th_bd = 20;
    if(   (v_2d(1,1)>wid+th_bd  &&  v_2d(2,1)>wid+th_bd &&  v_2d(3,1)>wid+th_bd ) ...
        || (v_2d(1,2)>hei+th_bd  &&  v_2d(2,2)>hei+th_bd &&  v_2d(3,2)>hei+th_bd) ...
        || (v_2d(1,1)< -th_bd &&  v_2d(2,1) <-th_bd  &&  v_2d(3,1) < -th_bd ) ...
        || (v_2d(1,2)<-th_bd  &&  v_2d(2,2)<-th_bd &&  v_2d(3,2)<-th_bd)  )
        continue;
    end
    
    %确保inc能覆盖每个像素,应该计算每个pixel的质心坐标，反求空间三角形采样点位置
    %这里简化为只扫描F的三角形，往图像上投影，可能会产生空洞
    dx = max(ceil(v_2d(:,1)))-min(floor(v_2d(:,1)));
    dy = max(ceil(v_2d(:,2)))-min(floor(v_2d(:,2)));
    inc=1.0/(max(dx,dy)+0.1) ;
    % inc = 0.08;
    d1 = v_(2,:)-v_(1,:);
    d2 = v_(3,:)-v_(1,:);
    [x,y] = meshgrid(0:inc:1,0:inc:1);x=x(:);y=y(:);
    idx = find((x+y)<1);
    len = numel(idx);
    pall = repmat(v_(1,:),len,1)+x(idx)*d1+y(idx)*d2;
    p2d = pall*kcam';
    
    p2d(:,1) = p2d(:,1)./p2d(:,3);
    p2d(:,2) = p2d(:,2)./p2d(:,3); % p2d(:,3)=p2d(:,3)./p2d(:,3);
    
    p2d = round(p2d);
    for k=1:size(p2d,1)
        r=p2d(k,2);if(r<1||r>hei), continue;end;
        c=p2d(k,1);if(c<1||c>wid), continue;end;
        depth(r,c)=min(depth(r,c),pall(k,3));
    end
end

if(1)
    depth = uint16(1000*depth+0.5);
else
    BF = 0.12 * fol; %120mm
    %depth = uint16(1000*depth+0.5);
    dis =  BF./(0.0001+depth) ;
    dis = round(dis*8)/8; 
    depth = BF./dis-0.0001; 
    depth = uint16(1000*depth+0.5);
end

return

%% test barycentric
a=[1,2,3;
    4,5,6;
    2.5,4,10;
    1,2,3];
d1=a(2,:)-a(1,:);
d2=a(3,:)-a(1,:);
pall=[];
inc=1/max([d1(1:2),d2(1:2)]);
inc=0.05;
[i,j]=meshgrid(0:inc:1,0:inc:1);i=i(:);j=j(:);
idx=(i+j)<=1;
pall=a(1,:)+i(idx)*d1+j(idx)*d2;
figure,plot3(a(:,1),a(:,2),a(:,3),'-')
hold on;
plot3(pall(:,1),pall(:,2),pall(:,3),'*')