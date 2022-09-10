%  by Wang Lin
%  2019.7.28
clear all
close all

datapath='..\ʵ��3_GICP\data\';
Kdp=load([datapath,'Kc.txt']);

range = [1,6,30];
%% ԭʼ�ļ������frameid 
depth_seq=[];
frameids=[];
for i=range(1):range(2):range(3)
    fname = [datapath,int2str(i),'.png'];
    dpz = single(imread(fname));
    %imshow(dpz,[])
    depth_seq = cat(3,depth_seq,dpz);
    frameids=[frameids,i];
end

[hei,wid,numpic] = size(depth_seq);
fprintf('%d pics loaded.\n',numpic);

%load pose��GT��ģ�ʹ�����������camera�������pose
%ͨ��ICP������ǵ�������ڵ�1֡��pose
pose_model_cam = [];
for i = range(1):range(2):range(3)
    pose_name =[datapath,'CameraRT',num2str(i),'.txt'];
    rt = load(pose_name);
    rt = [rt;0,0,0,1];
    rt(1:3,4) = rt(1:3,4)*1000;%mm
    pose_model_cam = cat(3,pose_model_cam,rt);
end

%1 ICP and Global ICP
% ʹ��ǰ�ογ̵ĳ��������Ƶ����Ƶ�pose
% ��Ҫ�����������Ҫ�������ģ�Ϳռ�pose_model_cam
% ��ҵ

%2 TSDF
res = [256;384;256]/1;    %wid hei thick
voxelsize =1400/res(1);
dim = res * voxelsize; %mm

%volume��ģ�����ĵ�offset
offset_model_vol = [-700;-900;-700];
 
clr = hot(30);
clr(1,:)=[1,1,1];

%% init coordinates of voxel grids
[x,y,z] = meshgrid( 1:res(1), 1:res(2), 1:res(3) );
x=single(x*voxelsize)+offset_model_vol(1);
y=single(y*voxelsize)+offset_model_vol(2);
z=single(z*voxelsize)+offset_model_vol(3);

Th_trunc = 4*voxelsize;
%% init volume by first depth frame
id_dep = 1;
depth = depth_seq(:,:,id_dep);
pose = pose_model_cam(:,:,id_dep);
[SDF0,Wt0] = TSDF_from_depth_and_xyz(depth,pose,x,y,z,Kdp,Th_trunc,[500,3500] );
for id_dep = 2:1:numpic
    id_dep
    depth = depth_seq(:,:,id_dep);
    pose = pose_model_cam(:,:,id_dep);
    %imshow(depth,[min(depth(:)),max(depth(:))],'Colormap',jet(255)) 
   [SDF1,Wt1] = TSDF_from_depth_and_xyz(depth,pose,x,y,z,Kdp,Th_trunc,[500,3500] );
   
   %% fusion...
    % copy new voxels, only w1 >0
    msk1 = (Wt0==0 & Wt1==1);
    SDF0(msk1) = SDF1(msk1);   Wt0(msk1) = 1;
    % weighted average both w>0
    msk2 = (Wt0>=1 & Wt1==1);
    SDF0(msk2) = SDF0(msk2).*single(Wt0(msk2)) + SDF1(msk2)*1;    
    SDF0(msk2) = SDF0(msk2)./single(Wt0(msk2)+1);    
    Wt0(msk2) = min(15,Wt0(msk2)+1);
end
fig = figure; slice(SDF0,res(1)/2,[],[]);colormap(fig,jet(16));axis equal;view(90,0)
fig = figure; slice(SDF0,[],res(2)*2/3,[]);colormap(fig,jet(16));axis equal;view(0,0)
fig = figure; slice(SDF0,[],[],res(3)/2);colormap(fig,jet(16));axis equal;view(0,-90)

fig = figure('Position',[1 1 1920 1080]);
hold on,
draw_volume_box(fig,[0;0;0], res );
[faces,verts] = isosurface(SDF0,0);
p = patch('Faces',int32(faces),'Vertices',single(verts),'FaceColor',clr(1,:),'EdgeColor','none');
view(0,-70); axis equal; 
camlight 
 
return
 
