function hcost = getHcost(x, y, goal, field)
    n = size(field);
    [goal_x, goal_y] = ind2sub(n,goal);
    hcost = abs(goal_x - x) + abs(goal_y - y);
    return;
end

