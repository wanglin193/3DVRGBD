%  by Wang Lin
%  2022.7.3
clear all
close all

datapath='data\';
Kdp=load([datapath,'Kc.txt']);

%% load A-pose model
[verts,faces] = read_obj([datapath,'smpl_male_2022.obj'] );
verts = 1000*verts';
faces = faces';

%模型自己转一下面向摄像机 z的负方向
verts(:,2:3) = -verts(:,2:3);

if(1) %add floor
    ymax = max(verts(:,2));
    s = 500;
    Vfloor=[-s,s,s,-s; ymax,ymax,ymax,ymax; -s,-s,s,s ]';
    Ffloor=[1,3,2;1,4,3]+6890;
    F = [faces;Ffloor];
    V = [verts;Vfloor];
else
    F =  faces;
    V =  verts ;
end

% show model
figure(1),hold on
if(1)
p = patch('Faces',F,'Vertices',V,'FaceColor','White','EdgeColor','none');
daspect([1,1,1])
p.FaceLighting = 'phong' ;
view(0,-90); axis tight
light('Position',[3000,3000,-3000],'Style','local','Color',[1,1,1])
end

%% 原始文件的序号frameid 
range = [1,1,30];
depth_seq=[];
frameids=[];
for i=range(1):range(2):range(3)
    fname = [datapath,int2str(i),'.png'];
    dpz = single(imread(fname));
    depth_seq = cat(3,depth_seq,dpz);
    frameids=[frameids,i];
end

[hei,wid,numpic] = size(depth_seq);
fprintf('%d pics loaded.\n',numpic);

%load pose的GT，模型创建的中心在camera坐标里的pose
%通常ICP计算的是点云相对于第1帧的pose
pose_model_cam = [];
for i = range(1):range(2):range(3)
    pose_name =[datapath,'CameraRT',num2str(i),'.txt'];
    rt = load(pose_name);
    rt = [rt;0,0,0,1];
    rt(1:3,4) = rt(1:3,4)*1000;%mm
    %inv的作用是变到model为中心的坐标系
    pose_model_cam = cat(3,pose_model_cam,inv(rt));
end
 
figure(1),hold on
for i=1:size(pose_model_cam,3)    
    cam_pose = pose_model_cam(:,:,i);
    draw_camera_pose( depth_seq(:,:,i),Kdp, cam_pose,0.5);   
end
axis equal
view(0,-60)


