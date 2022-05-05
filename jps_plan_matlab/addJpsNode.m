function [openList, camefrom, judge, field] = addJpsNode(jps_index,pre_gcost, cost, openList, camefrom, next_index, field)
    if camefrom(next_index) ~= -1
        judge = 1; %跳点已经被选过，但是仍然表示找到了跳点
         return;
    end
    openList = [openList; next_index, pre_gcost, cost]
    %max_num = max(field, [], 'ALL');
    field(next_index) = 0.5;
    camefrom(next_index) = jps_index;
    judge = 1;
end

