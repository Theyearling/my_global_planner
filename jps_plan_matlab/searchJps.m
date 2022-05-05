function [camefrom, openList, judge, field] = searchJps(jps_index, pre_x, pre_y, dx, dy, pre_gcost, camefrom, field, openList, goal)
    next_x = pre_x + dx;
    next_y = pre_y + dy;
    n = length(field);
    if ~isAvailable(next_x, next_y, field)
        judge = false;
        return;
    else
        next_index = sub2ind([n,n],next_x, next_y);
    end
    if next_index == goal
        % opList add
        [openList, camefrom, judge, field] = addJpsNode(jps_index,pre_gcost, pre_gcost, openList, camefrom, next_index, field);
        return;
    end
    if dx ~= 0 && dy ~= 0
        pre_gcost = pre_gcost + 1.4;
        % ¶Ô½ÇËÑË÷
        % ²»ÄÜ´©Ô½ÕÏ°­Îï
        if ~isAvailable(pre_x, next_y, field) && ~isAvailable(next_x, pre_y, field)
            judge = false;
            return;
        end
        % ×ÝÏòËÑË÷
        [camefrom, openList, judgex, field] = searchJps(next_index, next_x, next_y, dx, 0, pre_gcost, camefrom, field, openList, goal);
        % ºáÏòËÑË÷
        [camefrom, openList, judgey, field] = searchJps(next_index, next_x, next_y, 0, dy, pre_gcost, camefrom, field, openList, goal);
        % V¼â½ÇÇé¿ö
        if isVangle(pre_x, pre_y, next_x, next_y, dx, dy, field) || judgex || judgey
            cost = pre_gcost + 10*getHcost(next_x, next_y, goal, field);
            [openList, camefrom, judge, field] = addJpsNode(jps_index, pre_gcost, cost, openList, camefrom, next_index, field);
            return;
        end   
    elseif dx ~= 0
        pre_gcost = pre_gcost + 1;
        % ×ÝÏòËÑË÷
        if ~isAvailable(next_x, next_y + 1, field) && isAvailable(next_x + dx, next_y, field) && isAvailable(next_x + dx, next_y + 1, field)
            cost = pre_gcost + 10*getHcost(next_x, next_y, goal, field);
            [openList, camefrom, judge, field] = addJpsNode(jps_index,pre_gcost, cost, openList, camefrom, next_index, field);
            return;
        elseif ~isAvailable(next_x, next_y - 1, field) && isAvailable(next_x + dx, next_y, field) && isAvailable(next_x + dx, next_y - 1, field)
            cost = pre_gcost + 10*getHcost(next_x, next_y, goal, field);
            [openList, camefrom, judge, field] = addJpsNode(jps_index,pre_gcost, cost, openList, camefrom, next_index, field);
            return;
        end
    elseif dy ~= 0
        % ºáÏòËÑË÷
        pre_gcost = pre_gcost + 1;
        if ~isAvailable(next_x + 1, next_y, field) && isAvailable(next_x, next_y + dy, field) && isAvailable(next_x + 1, next_y + dy, field)
            cost = pre_gcost + 10*getHcost(next_x, next_y, goal, field);
            [openList, camefrom, judge, field] = addJpsNode(jps_index,pre_gcost, cost, openList, camefrom, next_index, field);
            return;
        elseif ~isAvailable(next_x - 1, next_y, field) && isAvailable(next_x, next_y + dy, field) && isAvailable(next_x - 1, next_y + dy, field)
            cost = pre_gcost + 10*getHcost(next_x, next_y, goal, field);
            [openList, camefrom, judge, field] = addJpsNode(jps_index,pre_gcost, cost, openList, camefrom, next_index, field);
            return;
        end
    else
        judge = false;
        return;
    end
    [camefrom, openList, judge, field] = searchJps(jps_index, next_x, next_y, dx, dy, pre_gcost, camefrom, field, openList, goal);
    return;

end

