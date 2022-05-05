% V¼â½ÇÇé¿ö
function judge = isVangle(pre_x, pre_y, next_x, next_y, dx, dy, field)
    if isAvailable(pre_x, next_y, field) && ~isAvailable(pre_x, next_y + dy, field) && ~isAvailable(next_x, next_y + dy, field)
        judge = 1;
        return;
    end
    if isAvailable(pre_y, next_x, field) && ~isAvailable(pre_y, next_x + dx, field) && ~isAvailable(next_y, next_x + dx, field)
        judge = 1;
        return;
    else
        judge = 0;
        return;
    end
end

