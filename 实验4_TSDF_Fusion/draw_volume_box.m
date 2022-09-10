function  draw_volume_box(fig, left_up_near, right_bot_far)
%  draw_volume_box(fig, left_up_near, right_bot_far)
%  by Wang Lin
%  2019.7.28

figure(fig);
hold on
axis equal
dim = right_bot_far - left_up_near;
pt = zeros(3,8);
pt(:,1) = left_up_near;
pt(:,2) = pt(:,1)+ [0;dim(2);0];
pt(:,3) = pt(:,1)+ [dim(1);dim(2);0];
pt(:,4) = pt(:,1)+ [dim(1);0;0];
 
pt(:,5) = pt(:,1)+ [0;0;dim(3)];
pt(:,6) = pt(:,1)+ [0;dim(2);dim(3)];
pt(:,7) = right_bot_far;
pt(:,8) = pt(:,1)+ [dim(1);0;dim(3)];
plot3(pt(1,[1:4,1,5:8,5]),pt(2,[1:4,1,5:8,5]),pt(3,[1:4,1,5:8,5]),'b-'); 
 
plot3(pt(1,[2,6]),pt(2,[2,6]),pt(3,[2,6]),'b-');
plot3(pt(1,[3,7]),pt(2,[3,7]),pt(3,[3,7]),'b-');
plot3(pt(1,[4,8]),pt(2,[4,8]),pt(3,[4,8]),'b-');
end

