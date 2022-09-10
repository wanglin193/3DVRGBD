clear all
close all
addpath('..\common')

myplot=@(x,colr) plot3(x(:,1),x(:,2),x(:,3),colr);
rootpath = 'data\';
datapath=[rootpath,'\'];
respath='output\';mkdir(respath);
    
out_ply_name = [respath,'D2moldel.ply'];

Kdp = load([rootpath,'Kc.txt']);

extname = '.png';
range = [1,6,30];

tic
%% 原始文件的序号frameid 
depth_seq=[];
frameids=[];
for i=range(1):range(2):range(3)
    fname = [datapath,int2str(i),extname];
    dpz = imread(fname);
    %imshow(dpz,[])
    depth_seq = cat(3,depth_seq,dpz);
    frameids=[frameids,i];
end

[hei,wid,numpic] = size(depth_seq);
fprintf('%d pics loaded.\n',numpic);

%% depth seq to PCs with verts and normals 
fprintf('depth to vertices and norms ... \n');
vts = cell(1,numpic);
nvs = cell(1,numpic);

depth_scale_factor = 1;
for i=1:numpic
    dp = depth_seq(:,:,i);
    [vts{i},nvs{i}] = dp2vn( single(dp),Kdp,depth_scale_factor ); 
end
clear depth_seq

%% step 1 pair wise ICP
need_check_loop = false;
ICP_option=struct( 'th_reject',60, 'Round', 15 );
PC = ICP_pairwise(vts,nvs,frameids,ICP_option,need_check_loop);
figure(9),title('Pairwise ICP') %ylim([-100,100])
plot_pcs(PC)

%% step 2 find scan patch pair list 
%PA = select_pc_pair(PC);
%simple way 
npc = numel(PC);
pairlist = nchoosek([1:npc],2);  % 比如  nchoosek([1:4],2)得到6种组合列表
select_neigh = 2; %1~3
d = pairlist(:,2)-pairlist(:,1);
id = d<select_neigh | d> npc-select_neigh;
PA.pair  = pairlist(id,:);

%% step 3 全局 point-plane error ICP
round_outter = 10;
round_inner = 5;
PC = global_icp_point2plane(PC,PA,round_outter,round_inner);

figure(10),title('Global ICP') %ylim([-100,100])
plot_pcs(PC)

t2 = toc
  
%% write to file
tic
idx = strfind(out_ply_name,'.');
idx = idx(end);
extname=out_ply_name(idx+1:end);
if(strcmp(extname,'xyz'))
    save_all_to_xyz( PC,out_ply_name);
elseif(strcmp(extname,'ply'))
    save_all_to_ply( PC,out_ply_name);
    idx=strfind(out_ply_name,'.ply');
    poissname = [out_ply_name(1:idx-1),'_poisson.ply'];
    command = ['PoissonRecon --in  ',out_ply_name,' --out ',poissname,' --depth 8'];
    [status,cmdout] = system(command)
end
fprintf('Write to file with Poisson reconstruction %4.3f sec.\n',toc);    

return

%% 
function po = apply_rt(p,M)
zv = ones(size(p,1),1);
po = [p,zv]*M';
po = po(:,1:3);
end

%% 在meshlab里泊松重建
function save_all_to_xyz( PC,fname )
numselect = numel(PC);
numv = 0;
for i = 1:numselect
    numv = numv+size(PC{i}.V,1);
end

f = fopen( fname, 'w');
for i = 1:numselect
    v1 = PC{i}.V;
    n1 = PC{i}.N;
    fprintf( f, '%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f \n', [v1,n1]');
end
fclose(f);
end
%%
function plot_pcs(PC)
myplot=@(x,colr) plot3(x(:,1),x(:,2),x(:,3),colr);
    hold on,axis equal,
    %title('Put all scans aligned')
    view(0,-25);
    for i=1:numel(PC)
        myplot(PC{i}.V,'.')
    end
end
%%
function save_all_to_ply( PC,fname)
numselect = numel(PC);
numv = 0;
for i = 1:numselect
    numv = numv+size(PC{i}.V,1);
end

f = fopen( fname, 'w');
fprintf( f, 'ply\nformat ascii 1.0\ncomment by wanglin\n');
fprintf( f, 'element vertex %d\n',numv);
fprintf( f, 'property float x\nproperty float y\nproperty float z\n');
fprintf( f, 'property float nx\nproperty float ny\nproperty float nz\nelement face 0\n');
fprintf( f, 'property list uchar int vertex_indices\n');
fprintf( f, 'end_header\n');

for i = 1:numselect
    v1 = PC{i}.V;
    n1 = PC{i}.N;
    fprintf( f, '%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f \n', [v1,n1]');
end
fclose(f);
end

