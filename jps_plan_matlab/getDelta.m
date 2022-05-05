function [dx,dy] = getDelta(jps_index, jps_x, jps_y, camefrom)
    pre_index = camefrom(jps_index);
    [pre_x, pre_y] = ind2sub([20,20],pre_index);
    dx = getOri(jps_x - pre_x);
    dy = getOri(jps_y - pre_y);
end

function delt = getOri(x)
    if x > 0
        delt = 1;
    elseif x < 0
        delt = -1;
    else
        delt = 0;
    end
end

