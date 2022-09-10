function [RT,Ra,Ta,vout] =  ICP_point2plane( v2,nv2,v1,nv1, option )
% input:  
%   point clound reference/target with normals: v1,nv1
%   source  with normals: v2,nv2
% output:
%   RT�ͱ任���source����vout

warning off MATLAB:nearlySingularMatrix

% ��������
if(isempty(option.Round))
    Round=20;
else
    Round=option.Round;
end

% ���þܾ���ֵ�����ڵ����һ����ֵ����ƥ���ԣ�����ĵ�λ��mm
if(isempty(option.th_reject))
    th_reject=16;
else
    th_reject=option.th_reject;
end

BaseThReject = th_reject;
Nv = size(v2,1);

% RT��ֵ
RT = eye(4);
Ra = zeros(3,3,Round);
Ta = zeros(3,Round);

% kd-tree
Mdl = createns(v1,'Distance','Euclidean');
p3d2 = v2;
nv2_trans = nv2;

%% ����
for iter=1:Round
    ThReject = BaseThReject;
    
    % �ܾ�������ֵ
    if(iter>round(Round/2))
        % ����ÿ�ε������ʵ���С�ܾ���ֵ
        BaseThReject = BaseThReject*0.8;
        % ����ֵ����һ�����ޣ����粻Ҫ����3mm����3mm���ڵĵ������ʵĽ��ڵ�
        ThReject = max(BaseThReject,3);
    end
    
    % step 1 of ICP: searching
    [id2,dis2] = knnsearch(Mdl,p3d2);
    
    % ��һ�����þܾ���ֵ�ķ�����������ƥ��������ֵ
    % ��������Ӧ��ֵ����ʱ����ʹ
    %ThReject = median(dis2);
    
    % �ҵ���Ӧ�Ľ��ڵ�����
    ind2 = find(dis2<ThReject);
    % ��̫�٣������ص�����̫�٣�ƥ��ʧ����
    if(numel(ind2)<10)
        disp('Too less points -- less tha 10 pts')
        break;
    end
    
    % �ҵ�ƥ���Ե���� ind1,ind2    
    ind1 = id2(ind2);   
    
    % �÷��ߵ����ƶ��ٴξܾ���
    ThNorm = 0.9;
    id_norm = find( dot(nv1(ind1,:),nv2_trans(ind2,:),2)>ThNorm );
    ind1 = ind1(id_norm);
    ind2 = ind2(id_norm);
    
    p3d2 = p3d2(ind2,:);    
    p3d1 = v1(ind1,:);
    pn1 =  nv1(ind1,:);        
    
    % step 2 of ICP: d[R,T]
    N = size(p3d1,1);
    % �������е��Ȩ��һ��ones(N,1)
    % ��dR,dTֻ��һ�ε���
    [dR,dT] = pnt2plane(p3d1,p3d2,pn1,ones(N,1));
    % R,T�����ֱ������ÿ�ε�����RT���ɼ�¼����
    Ra(:,:,iter) = dR;
    Ta(:,iter) = dT;
    
    % ��������RT,4*4��������
    RT = [dR,dT;
          0,0,0,1] * RT;
    
    % ����source���Ƶ�����
    p3d2 = [v2,ones(Nv,1)]*RT';
    p3d2 = p3d2(:,1:3);
    
    % ����source���Ƶķ��߷���    
    nv2_trans = nv2*RT(1:3,1:3)';
    
    % �����㹻����ʾ������������ǰ�˳�����
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

