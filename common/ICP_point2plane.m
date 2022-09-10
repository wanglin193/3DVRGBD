function [RT,Ra,Ta,vout] =  ICP_point2plane( v2,nv2,v1,nv1, option )
% input:  
%   point clound reference/target with normals: v1,nv1
%   source  with normals: v2,nv2
% output:
%   RT和变换后的source点云vout

warning off MATLAB:nearlySingularMatrix

% 迭代次数
if(isempty(option.Round))
    Round=20;
else
    Round=option.Round;
end

% 设置拒绝阈值，近邻点大于一定阈值不算匹配点对，这里的单位是mm
if(isempty(option.th_reject))
    th_reject=16;
else
    th_reject=option.th_reject;
end

BaseThReject = th_reject;
Nv = size(v2,1);

% RT初值
RT = eye(4);
Ra = zeros(3,3,Round);
Ta = zeros(3,Round);

% kd-tree
Mdl = createns(v1,'Distance','Euclidean');
p3d2 = v2;
nv2_trans = nv2;

%% 迭代
for iter=1:Round
    ThReject = BaseThReject;
    
    % 拒绝距离阈值
    if(iter>round(Round/2))
        % 例如每次迭代都适当减小拒绝阈值
        BaseThReject = BaseThReject*0.8;
        % 给阈值设置一个下限，比如不要低于3mm，即3mm以内的点才算合适的近邻点
        ThReject = max(BaseThReject,3);
    end
    
    % step 1 of ICP: searching
    [id2,dis2] = knnsearch(Mdl,p3d2);
    
    % 另一种设置拒绝阈值的方法：即所有匹配距离的中值
    % 好像是适应阈值但有时不好使
    %ThReject = median(dis2);
    
    % 找到对应的近邻点的序号
    ind2 = find(dis2<ThReject);
    % 点太少，可能重叠区域太少，匹配失败了
    if(numel(ind2)<10)
        disp('Too less points -- less tha 10 pts')
        break;
    end
    
    % 找到匹配点对的序号 ind1,ind2    
    ind1 = id2(ind2);   
    
    % 用法线的相似度再次拒绝。
    ThNorm = 0.9;
    id_norm = find( dot(nv1(ind1,:),nv2_trans(ind2,:),2)>ThNorm );
    ind1 = ind1(id_norm);
    ind2 = ind2(id_norm);
    
    p3d2 = p3d2(ind2,:);    
    p3d1 = v1(ind1,:);
    pn1 =  nv1(ind1,:);        
    
    % step 2 of ICP: d[R,T]
    N = size(p3d1,1);
    % 设置所有点的权重一样ones(N,1)
    % 求dR,dT只做一次迭代
    [dR,dT] = pnt2plane(p3d1,p3d2,pn1,ones(N,1));
    % R,T增量分别给出，每次迭代的RT都可记录下来
    Ra(:,:,iter) = dR;
    Ta(:,iter) = dT;
    
    % 增量更新RT,4*4矩阵连乘
    RT = [dR,dT;
          0,0,0,1] * RT;
    
    % 更新source点云的坐标
    p3d2 = [v2,ones(Nv,1)]*RT';
    p3d2 = p3d2(:,1:3);
    
    % 更新source点云的法线方向    
    nv2_trans = nv2*RT(1:3,1:3)';
    
    % 距离足够近表示收敛，可以提前退出迭代
    if( mean(dis2)<1 )
        fprintf('Quit at round %d\n',iter);
        break;
    end
end
vout = p3d2;

end

%% point to plane
function [R,T] = pnt2plane(q,p,nq,weights)
nq = nq .* repmat(weights,1,3);
c = cross(p,nq);
cn = [c,nq];

%<dP,n> of points
b = dot((p-q),nq,2);

% solve cn*X = -b
C = cn'*cn ;
b = -cn'*b;
X = C\b;

T = X(4:6);
if(1)
    cx = cos(X(1)); cy = cos(X(2)); cz = cos(X(3));
    sx = sin(X(1)); sy = sin(X(2)); sz = sin(X(3));
    R = [cy*cz cz*sx*sy-cx*sz cx*cz*sy+sx*sz;
        cy*sz cx*cz+sx*sy*sz cx*sy*sz-cz*sx;
        -sy cy*sx cx*cy];
else
    [R, dRdr] = so3exp(X(1:3)');
end

end

