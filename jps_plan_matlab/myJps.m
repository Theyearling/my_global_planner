%matlab��ʼ��

clc;             %�������ڵ�����
clear all;       %��������ռ�����б�������������MEX�ļ�
close all;       %�ر����е�figure����

n = 20;
wallpercent = 0.4;
[field, start, goal] = initialize(n,wallpercent);
camefrom = -1*ones(n);
openList = [start, 0, 0]
camefrom(start) = 0;

axishandle = createFigure(field,start,goal);

while ~isempty(openList)
    pause(1);
    [min_cost, col] = min(openList(:,3));
    jps_index = openList(col, 1);
    pre_gcost = openList(col, 2);
    [jps_x, jps_y] = ind2sub([n,n],jps_index);
    %��������
    if camefrom(jps_index) == 0
        for ii = [-1, 0 , 1]
            for jj = [-1, 0, 1]
                if ii == 0 && jj == 0
                    continue;
                else
                    [camefrom, openList, judge, field] = searchJps(jps_index, jps_x, jps_y, ii, jj, pre_gcost, camefrom, field, openList, goal);
                end
            end
        end
    else
        [dx, dy] = getDelta(jps_index, jps_x, jps_y, camefrom);
        delta = [dx, dy];
        delta = getForceNer(jps_x,jps_y,dx,dy,camefrom,field,delta);
        [a,b] = size(delta);
        for col_i=1:a
            [camefrom, openList, judge, field] = searchJps(jps_index,jps_x,jps_y,delta(col_i,1),delta(col_i,2),pre_gcost,camefrom,field,openList,goal);
        end
        %��ǿ���ھӵ�����
        
    end
    %ȡ����С���۽ڵ�
    openList
    if col > 1 && col < length(openList)
        disp('�м�')
        openList = [openList(1: col - 1,1: 3); openList(col + 1: end,1: 3)]
    elseif col == 1
        disp('col == 1')
        openList = [openList(2: end,1: 3)]
    else
        disp('��β')
        openList = [openList(1: end - 1,1: 3)]
    end
    
    %��ͼ
    if isempty(openList) 
        disp('No path');
        return; 
    end
    set(axishandle,'CData',[field field(:,end); field(end,:) field(end,end)]);
    set(gca,'CLim',[0 1*max(field(field < Inf))]);
    drawnow; 
end


  
