clear all
close all

addpath('..\common\')
path = '..\data\rgbd_gongzai\';
path = '..\data\opencv_odometry\';

dep_path = [path,'\'];
rgb_path = [path,'\'];

% intrinsic
Kc=[475.722, 0.0, 310.241
    0.0, 475.722, 246.003
    0.0,  0.0,    1.0];
 
% depth
dp{1} = imread([dep_path,'depth_00000.png']);
dp{2} = imread([dep_path,'depth_00002.png']);
% rgb
im{1} = imread([rgb_path,'image_00000.png']);
im{2} = imread([rgb_path,'image_00002.png']);

%% 稍微增加些难度，图像颠倒一下
if(1)
    im{2} = imrotate(im{2},180);
    dp{2} = imrotate(dp{2},180);
end

%%
%Detect features in both images.
imgray1=rgb2gray(im{1});
imgray2=rgb2gray(im{2});


figure,imshow([imgray1,imgray2],[]),title('Grayimage pair')

pts1 = detectSURFFeatures(imgray1,'NumScaleLevels',4,'MetricThreshold',1800,'NumOctaves',4); 
pts2 = detectSURFFeatures(imgray2,'NumScaleLevels',4,'MetricThreshold',1800,'NumOctaves',4); 

%Extract feature descriptors.
[pf1,validPts1] = extractFeatures(imgray1, pts1);
[pf2,validPts2] = extractFeatures(imgray2, pts2);

%Match features by using their descriptors.
indexPairs = matchFeatures(pf1, pf2);

%Retrieve locations of corresponding points for each image.
matched1 = validPts1(indexPairs(:,1));
matched2 = validPts2(indexPairs(:,2));

%Show putative point matches.
figure;
showMatchedFeatures(im{1},im{2},matched1,matched2,'montage');
title('Putatively matched points (including outliers)');

% get z from depth
z1 = getz(dp{1},matched1.Location);
z2 = getz(dp{2},matched2.Location);
 
idx_valid = find(z1>0 & z2>0);
matched1 = matched1(idx_valid);
matched2 = matched2(idx_valid);

figure,showMatchedFeatures(dp{1},dp{2},matched1,matched2,'montage');
title('Matched valid points')

%[x,y,z]
p2d1 = matched1.Location; Z1=z1(idx_valid);
p2d2 = matched2.Location; Z2=z2(idx_valid);
v1 = [(p2d1(:,1)-Kc(1,3)).*Z1/Kc(1,1),(p2d1(:,2)-Kc(2,3)).*Z1/Kc(2,2),Z1];
v2 = [(p2d2(:,1)-Kc(1,3)).*Z2/Kc(1,1),(p2d2(:,2)-Kc(2,3)).*Z2/Kc(2,2),Z2];

% M-estimator sample consensus (MSAC) algorithm
estimatedTform = estimateGeometricTransform3D( v1, v2, 'rigid','MaxNumTrials', 5000, 'Confidence',90, 'MaxDistance',4); %
disp(estimatedTform.T')  

% V and N
downsize_factor = 2;
[vts{1},nvs{1}] = dp2vn( single(dp{1}),Kc,downsize_factor );
[vts{2},nvs{2}] = dp2vn( single(dp{2}),Kc,downsize_factor );

%  Use the estimated RT to  transform vts{2} back to the vts{1}
rt2_1 = inv(estimatedTform.T');
pt2Tformed = ApplyRT(vts{2},rt2_1);
nv2 = ApplyR(nvs{2},rt2_1(1:3,1:3)); 
write_xyz_vn('v1.xyz', vts{1},nvs{1});
write_xyz_vn('v2_byRT.xyz', pt2Tformed,nv2);

figure
plot3(vts{1}(:,1),vts{1}(:,2),vts{1}(:,3),'.');hold on
plot3(pt2Tformed(:,1),pt2Tformed(:,2),pt2Tformed(:,3),'r.');
axis equal
title('PC aligned')
return

%%
function z=getz(dp,pxy)
p1=round(pxy);
z=zeros(size(p1,1),1);
for i=1:size(p1,1)
    z(i)=dp(p1(i,2),p1(i,1));
end 
end

function po = ApplyRT(p,M)
zv = ones(size(p,1),1);
po = [p,zv]*M';
po = po(:,1:3);
end

function po = ApplyR(p,R)
 po = p*R'; 
end
 