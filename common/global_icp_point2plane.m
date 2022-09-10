function PC = global_icp_point2plane(PC,PA,round_outter,round_inner)
num_match_pc = size(PA.pair,1);
npc=numel(PC);
%%
for loop_outter=1:round_outter
    if(loop_outter>0)
        %选近邻点
        dist_match = 0;
        for i = 1:num_match_pc
            pcid1  =  PA.pair(i,1);
            pcid2  =  PA.pair(i,2);
            
            [id2,dis2]=knnsearch(PC{pcid1}.V,PC{pcid2}.V );
            
            dist_match = dist_match + sum(dis2.^2);
            if(0)
                [dis_sort,~]=sort(dis2);
                idx=floor(numel(dis_sort)*0.8);
                ThReject = dis_sort(idx);
            else
                ThReject = median(dis2);
            end
            
            ind2 = find(dis2<ThReject);
            ind1 = id2(ind2);
            
            ThNorm = 0.98 ;
            id_norm =find(dot(PC{pcid1}.N(ind1,:),PC{pcid2}.N(ind2,:),2)>ThNorm);
            ind1 = ind1(id_norm);
            ind2 = ind2(id_norm);
            
            % fprintf('%4.3f point select\n',numel(ind2)/numel(dis2));
            PA.coresp{i} = int32([ind1,ind2]);
        end
        fprintf('dist= %4.2f, ', sqrt(dist_match) );
    end
    
    row_id_e{1}=size(PA.coresp{1},1);
    for i = 2:num_match_pc
        row_id_e{i} = row_id_e{i-1}+size(PA.coresp{i},1);
    end
    fprintf('%d match pts found in loop %d.\n', row_id_e{i},loop_outter);
    
    %construct Matrix by PA and PC for
    %Ax=B
    A = zeros(row_id_e{num_match_pc},npc*6,'single');
    B = zeros(row_id_e{num_match_pc},1,'single');
    for loop_inner=1:round_inner
        for i = 1:num_match_pc
            pcid = PA.coresp{i};
            num_coresp = size(PA.coresp{i},1);
            
            idm = PA.pair(i,1);
            idn = PA.pair(i,2);
            Pm = PC{idm}.V(pcid(:,1),:);
            Pn = PC{idn}.V(pcid(:,2),:);
            Nm = PC{idm}.N(pcid(:,1),:);
            
            [J1,J2,b] = create_J_block(Pm,Pn,Nm);
            
            if(i==1)
                row_id_s=1;
            else
                row_id_s=row_id_e{i-1}+1;
            end
            A(row_id_s:row_id_e{i},idm*6-5:idm*6) = J1;
            A(row_id_s:row_id_e{i},idn*6-5:idn*6) = J2;
            B(row_id_s:row_id_e{i}) = b;
        end
        
        id_fix_pc = 1;
        
        Af = A;
        Af(:,id_fix_pc*6-5:id_fix_pc*6)=[];
        AtA = Af'*Af;        
        X =  (AtA  )\(Af'*B);
        
        %parse rt for all
        n=1;
        dRT=cell(npc,1);
        for i=1:npc
            if(i==id_fix_pc)
                dRT{i}=eye(4);
            else
                dRT{i} = rt2RT(X(n*6-5:n*6));
                n=n+1;
            end            
        end
        
        %apply rt to all pc
        for i=1:npc
            if(i==id_fix_pc)
                continue;  
            end
            rt = dRT{i};
            PC{i}.V = apply_rt(PC{i}.V,rt);
            PC{i}.N = PC{i}.N * rt(1:3,1:3)';
            PC{i}.RT = rt*PC{i}.RT;
        end
    end
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%se3->SE3
function RT = rt2RT(X)
RT = eye(4);
if(numel(X)~=6)
    return
end
RT(1:3,4)=X(4:6);
if(1)
    cx = cos(X(1)); cy = cos(X(2)); cz = cos(X(3));
    sx = sin(X(1)); sy = sin(X(2)); sz = sin(X(3));
    RT(1:3,1:3) = [cy*cz cz*sx*sy-cx*sz cx*cz*sy+sx*sz;
        cy*sz cx*cz+sx*sy*sz cx*sy*sz-cz*sx;
        -sy cy*sx cx*cy];
else
    [RT(1:3,1:3),~] = so3exp(X(1:3)');
end
end

%% sparse block 有2个block 对应12个参数
function [J1,J2,b] = create_J_block(Pm,Pn,Nm)
J1=  [cross(Pm,Nm),Nm];
J2=  [-cross(Pn,Nm),-Nm];
b = -dot((Pm-Pn),Nm,2);
end

function po = apply_rt(p,M)
zv = ones(size(p,1),1);
po = [p,zv]*M';
po = po(:,1:3);
end