close all
Kc = load("data\Kc.txt");
dp = imread('data\4.png');

%dp = imread('frame-000000.depth.png');
%Kc=[520,0,320;0,520,240;0,0,1];

[H,W,~]=size(dp);
dp = single(dp);
%
[x2d,y2d]=meshgrid(1:W,1:H);

X=(x2d-Kc(1,3)).*dp/Kc(1,1);
Y=(y2d-Kc(2,3)).*dp/Kc(2,2);
 
% given a predict plane
ABCD = [0,-1,0,800];
pt = [X(:),Y(:),dp(:)];
dis=ABCD(1)*pt(:,1)+ABCD(2)*pt(:,2)+ABCD(3)*pt(:,3)+ABCD(4);
% select subset of point cloud
id=abs(dis)<400&dp(:)>0;
pt=pt(id,:);

% id=abs(ABCD(1)*pt(:,1)+ABCD(2)*pt(:,2)+ABCD(3)*pt(:,3)+ABCD(4))<100;
% figure,hold on
% plot3(pt(:,1),pt(:,2),pt(:,3),'.'),axis equal
% plot3(pt(id,1),pt(id,2),pt(id,3),'r.')
% view(0,-70)

%weighted MSE
wei = ones(size(pt,1),1);
for round=1:10
    ABCD = fit_plane(pt,wei);
    dis = ABCD(1)*pt(:,1)+ABCD(2)*pt(:,2)+ABCD(3)*pt(:,3)+ABCD(4);
    wei = 1./(dis.^2+600); wei = 1*wei/sum(wei);
end

id=abs(ABCD(1)*pt(:,1)+ABCD(2)*pt(:,2)+ABCD(3)*pt(:,3)+ABCD(4))<10;
figure,hold on
plot3(pt(:,1),pt(:,2),pt(:,3),'.'),axis equal
plot3(pt(id,1),pt(id,2),pt(id,3),'g.')
view(0,-70)

function ABCD = fit_plane(pt,wei)
pnum=size(pt,1);
A=[pt,ones(pnum,1)];
W=repmat(wei,1,4);
[u,~,~]=svd(A'*(W.*A));
%last column of u
ABCD = u(:,4);
ABCD = ABCD/norm(ABCD(1:3));
end