clear all
close all

addpath('..\common\')
path = '..\data\rgbd_gongzai\';
dep_path = [path,'depth\'];
rgb_path = [path,'rgb\'];

%相机内参
Kc=load([path,'K_rgb.txt']);

 
% 深度序列
dp{1} = imread([dep_path,'27.png']);
dp{2} = imread([dep_path,'47.png']);

im{1} = imread([rgb_path,'27.png']);
im{2} = imread([rgb_path,'47.png']);

%remove background  
thd = 900;
dp{1}=single(dp{1}).*(dp{1}<thd);
dp{2}=single(dp{2}).*(dp{2}<thd);

figure
imshow([im{1},im{2}],[]),title('rgb pair')

figure
imshow([dp{1},dp{2}],[]),title('depth pair')

% V and N
downsize_factor = 6;
[vts{1},nvs{1}] = dp2vn( single(dp{1}),Kc,downsize_factor );
[vts{2},nvs{2}] = dp2vn( single(dp{2}),Kc,downsize_factor );

pc{1} = pointCloud(vts{1},'Normal',nvs{1});
pc{2} = pointCloud(vts{2},'Normal',nvs{2});
figure
pcshowpair(pc{1},pc{2},'MarkerSize',150,'BackgroundColor',[1,1,1],'VerticalAxis','Y','VerticalAxisDir','Down')
title("PC pair")
%%
% extractFPFHFeatures：从点云中提取特征
% pcmatchfeatures：特征匹配
% estimateGeometricTransform3D：匹配特征估计刚性转换
 
pf1 = extractFPFHFeatures(pc{1},'NumNeighbors',100);  
pf2 = extractFPFHFeatures(pc{2},'NumNeighbors',100); %'NumNeighbors',250 

[matchingPairs,scores] = pcmatchfeatures(pf1,pf2, pc{1},pc{2},"Method","Approximate"); %'Exhaustive' (default) | 'Approximate'
length( matchingPairs )
assert( size(matchingPairs,1)>3 )

pts1 = select(pc{1},matchingPairs(:,1));
pts2 = select(pc{2},matchingPairs(:,2));

figure
pcshowMatchedFeatures(pc{1},pc{2},pts1,pts2, "Method","montage")
%xlim([-40 210])
%ylim([-50 50])
title("Matched Points")
 
v1 = pts1.Location;
v2 = pts2.Location;

estimatedTform = estimateGeometricTransform3D( v1, v2, 'rigid','MaxNumTrials', 5000, 'Confidence',90, 'MaxDistance',10); %
disp(estimatedTform.T) 
 
%Use the estimated transformation to retransform ptCloudTformed back to the initial point cloud.
pt2Tformed = pctransform(pc{2},invert(estimatedTform));

%Visualize the two point clouds.
figure 
pcshowpair(pc{1},pt2Tformed,'MarkerSize',150,'BackgroundColor',[1,1,1],'VerticalAxis','Y','VerticalAxisDir','Down')
title("Aligned Point Clouds")


