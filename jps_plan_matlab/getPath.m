function path = getPath(camefrom,goal,start)
    [px, py] = ind2sub([20,20],goal);
    path = [px, py];
    index = camefrom(goal);
    while index ~= -1 && index ~= start
        [px, py] = ind2sub([20,20],index);
        path = [path; px, py];
        index = camefrom(index);
    end
    [px, py] = ind2sub([20,20],index);
    path = [path; px, py];
end

