function PC = ICP_pairwise(vts,nvs,frameids,ICP_option,need_check_loop)
myplot = @(x,colr) plot3(x(:,1),x(:,2),x(:,3),colr);

%%
verbose = 0;
rtall = eye(4);
cameratrack = rtall(1:3,4);
guessRT =eye(4);
init_ax=[];
rotate_half_loop=false;
isloop = false;
pose_cam_model = rtall;
num_loop =-1;

fprintf('pairwise ICP .... \n')
%ICP_option=struct( 'th_reject',20, 'Round', 10 );

theta_old = 0;
for i = 1:numel(vts)-1
    %loop detection by angle of R
    if(need_check_loop && isloop)
        disp('  loop detected.')
        break;
    end

    vertex1 = vts{i}; nv1 = nvs{i};
    vertex2 = vts{i+1}; nv2 = nvs{i+1};

    %% target 3-layer map
    vertex2 = apply_rt(vertex2,guessRT);
    [dRT,Ra,Ta,pv] = ICP_point2plane(vertex2,nv2,vertex1,nv1,ICP_option);
    RT = dRT*guessRT;

    %2-->1
    rtall = rtall*(RT);

    R = double(rtall(1:3,1:3));
    [u,~,v]=svd(R); R = u*v';
    rtall(1:3,1:3) = R;
    [omega,theta,ax] = so3log(R);
    if(isempty(init_ax))
        init_ax = ax;
    end

    %if axis change direcition
    check_ax = dot(ax,init_ax);
    if(check_ax<0)
        ax=-ax;
        theta=2*pi-theta;
    end

    if(verbose)
        figure(999),hold on,axis equal
        plot3([0,ax(:,1)],[0,ax(:,2)],[0,ax(:,3)],'-*');
    end

    pose_cam_model = cat(3,pose_cam_model,(rtall));

    % Frobenius norm, sqrt(sum(diag(X'*X))).
    X=rtall(1:3,1:3) - eye(3);
    dis_f =  norm(X(:));

    fprintf('angle %4.2f\n ',theta*180/pi);

    %check loop or not
    if( theta>pi*0.9 && ~rotate_half_loop && i>4  )
        rotate_half_loop=true;
    end
    if( theta_old-theta>pi && rotate_half_loop )
        isloop=true;
    end

    %guess for next
    guessRT = RT;
    theta_old = theta;
    if(verbose)
        figure,hold on
        myplot( vertex1,'b.');
        myplot( pv,'g.');
        legend('target/fixed','source/moved')
        axis equal,grid on
        view(0,-85);
    end
end
numselect = size(pose_cam_model,3);
fprintf('num of pics select = %d \n',numselect);

%% put all depth into model space
n = 1;
%% 注意！这里选关键帧可以是跳帧的
leap=1;
for i = 1:leap:numselect
    RT = pose_cam_model(:,:,i);
    v1 = apply_rt(vts{i},RT);
    n1 = nvs{i}*RT(1:3,1:3)';

    idx = sum(n1.*n1,2)>0.99;
    v1 = v1(idx,:);
    n1 = n1(idx,:);

    if(0)
        fname = ['myicp_data\myicp_',int2str(i),'.xyz'];
        write_xyz(fname,v1,n1);
    end

    PC{n}.V = v1;
    PC{n}.N = n1;
    PC{n}.RT = RT;
    PC{n}.frameid = frameids(i);
    n = n+1;
end

if(0)
    idx=strfind(out_ply_name,'.');
    out_name = [out_ply_name(1:idx-1),'_pairwise_aligned.xyz'];
    save_all_to_xyz( PC,out_name);
end
if(0)
    figure(9999),hold on,axis equal,
    title('Put all scans aligned')
    view(0,-25);
    for i=1:numel(PC)
        figure(9999),myplot(PC{i}.V,'.')
    end
end
end

function po = apply_rt(p,M)
zv = ones(size(p,1),1);
po = [p,zv]*M';
po = po(:,1:3);
end
