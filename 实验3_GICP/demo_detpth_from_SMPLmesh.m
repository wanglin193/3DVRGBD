%加载SMPL刚体人体模型，2d投影得到深度图
clear all
close all
addpath('..\common\')

basename = 'smpl_male_2022';
path = ['data/']; mkdir(path);

%% load A-pose model
[verts,faces] = read_obj([path,'smpl_male_2022.obj'] );
verts=verts';
faces=faces';

%模型自己转一下面向摄像机 z的负方向
verts(:,2:3) = -verts(:,2:3);

if(1) %add floor
    ymax = max(verts(:,2));
    s = 0.5;
    Vfloor=[-s,s,s,-s; ymax,ymax,ymax,ymax; -s,-s,s,s ]';
    Ffloor=[1,3,2;1,4,3]+6890;
    F = [faces;Ffloor];
    V = [verts;Vfloor];
else
    F =  faces;
    V =  verts ;
end

figure(1),hold on
p = patch('Faces',F,'Vertices',V,'FaceColor','White','EdgeColor','none');
daspect([1,1,1])
p.FaceLighting = 'phong' ;
view(0,-90); axis tight
light('Position',[-1,1,-4],'Style','local','Color',[1,1,1])

%% 深度序列
wid = 480; hei = 640; fol = 520;
kcam = [fol,0,wid/2;0,fol,hei/2;0,0,1];
fid = fopen([path,'Kc.txt'],'w');
    fprintf(fid,'%3.1f, %3.1f, %3.1f\n',kcam');
fclose(fid);

R = eye(3); T = [0,-0.3,2.0]';
Rt = [R,T];
Rt_cam_model = [];
numpic=30;
RT_all =[];
for i=1:numpic
    Rt = [R,T];
    % center at model, inverse of RTs(center at cameras)
    invRT = inv([Rt;0,0,0,1]);
    Rt_cam_model = cat(3,Rt_cam_model,invRT);

    depth = gen_depth_from_mesh(V,F,wid,hei,fol,Rt);

    if(1)
        %depth to file
        dpname = [path,num2str(i),'.png'];
        imwrite(depth,dpname);
        
    	%camera pose to file
        rtname =[path,'CameraRT',num2str(i),'.txt'];
        fid = fopen(rtname,'w');
        fprintf(fid,'%g,%g,%g,%g\r\n',Rt');
        fclose(fid);
    end

    figure(4),
    %imshow(depth,[])
    imshow(depth,[min(depth(:)),max(depth(:))],'Colormap',jet(255))

    RT_all = cat(3,RT_all,[Rt;0,0,0,1]);

    % customize camera trajectory
    dT=[-0.01;0.02]/3;
    if(i<numpic/2)
        T(2:3) = T(2:3)-dT;
    else
        T(2:3) = T(2:3)+dT;
    end

    th1 = (1*pi/4)/(numpic-1);
    dRx = [ 1, 0,0; 0,cos(th1),sin(th1); 0,-sin(th1),cos(th1)];
    R = dRx*R;

    th2 = 2*pi/(numpic-1);
    dRy = [cos(th2),0,sin(th2); 0,1,0; -sin(th2),0,cos(th2)];
    R = dRy*R;
end

%% draw cameras
figure(1)
len=0.2;
frame = [ 0,0,0,1;  len,0,0,1;  0,len,0,1;  0,0,len,1]';

for i=1:size(Rt_cam_model,3)-1
    fr = Rt_cam_model(:,:,i)*frame;
    plot3(fr(1,[1,2]),fr(2,[1,2]),fr(3,[1,2]),'r');
    plot3(fr(1,[1,3]),fr(2,[1,3]),fr(3,[1,3]),'g');
    plot3(fr(1,[1,4]),fr(2,[1,4]),fr(3,[1,4]),'b');
end
view(0,-70)

return

