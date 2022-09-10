function PA = select_pc_pair(PC)
npc=numel(PC);
pairlist = nchoosek([1:npc],2);
%pairlist=[1:npc;2:npc,1]';

%筛选重合度大的mesh pair index
%大的回环需要调整matching pair的可以挪到 outter loop里
n=1;
for i = 1:size(pairlist,1)
    pcid1 = pairlist(i,1);
    pcid2 = pairlist(i,2);
    select_neigh = 2;%1~3
    if( pcid2-pcid1>select_neigh  && (pcid2-pcid1)<npc-select_neigh )
        continue;
    end
    
    [id2,dis2] = knnsearch(PC{pcid1}.V,PC{pcid2}.V );

    ThReject = 20;%mm
    ind2 = find(dis2<ThReject);

    if(numel(ind2)<0.4*numel(id2))
        disp('Too less nearest points ')
        continue;
    end
    ind1 = id2(ind2);
    
    PA.coresp{n} = int32([ind1,ind2]);
    PA.pair(n,:) = pairlist(i,:);    
    n = n+1;
end
num_match_pc = numel(PA.coresp);
fprintf('Select PC %d pair indices:\n',num_match_pc);
fprintf('%3d',PA.pair(:,1));fprintf('\n');
fprintf('%3d',PA.pair(:,2));fprintf('\n');
end