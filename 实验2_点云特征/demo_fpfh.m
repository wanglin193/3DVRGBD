clear all
close all

ptObj = pcread('..\bunny_coarse.ply');

%降采样
ptCloudIn = pcdownsample(ptObj,'gridAverage',0.05);

%计算FPFH
keyInds = [1 950];
features = extractFPFHFeatures(ptCloudIn,keyInds);

ptKeyObj = pointCloud(ptCloudIn.Location(keyInds,:),'Color',[255 0 0;0 0 255]);

figure, pcshow(ptObj),
title('Selected Indices on Point Cloud')
hold on,
pcshow(ptKeyObj,'MarkerSize',1000), 

figure, ax1 = subplot(2,1,1);
bar(features(1,:),'FaceColor',[1 0 0])
title('FPFH Descriptors of Selected Indices')
ax2 = subplot(2,1,2);
bar(features(2,:),'FaceColor',[0 0 1])
linkaxes([ax1 ax2],'xy')