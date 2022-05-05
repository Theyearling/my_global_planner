function delta = getForceNer(cur_x,cur_y,dx,dy,camefrom,field,delta)
    %x÷·
    if isAvailable(cur_x + dx, cur_y + 1, field)
        next_index = sub2ind([20,20],cur_x + dx, cur_y + 1);
         if ~isAvailable(cur_x, cur_y + 1, field) && isAvailable(cur_x + dx, cur_y, field) && camefrom(next_index) == -1
             delta = [delta; dx, 1];
         end
    end
    if isAvailable(cur_x + dx, cur_y - 1, field)
        next_index = sub2ind([20,20],cur_x + dx, cur_y - 1);
        if ~isAvailable(cur_x, cur_y - 1, field) && isAvailable(cur_x + dx, cur_y, field) && camefrom(next_index) == -1
            delta = [delta; dx, -1];
        end
    end
    %y÷·
    if isAvailable(cur_x + 1, cur_y + dy, field)
        next_index = sub2ind([20,20],cur_x + 1, cur_y + dy);
        if ~isAvailable(cur_x + 1, cur_y, field) && isAvailable(cur_x, cur_y + dy, field) && camefrom(next_index) == -1
            delta = [delta; 1, dy];
        end
    end
    if isAvailable(cur_x - 1, cur_y + dy, field)
        next_index = sub2ind([20,20],cur_x - 1, cur_y + dy);
        if ~isAvailable(cur_x - 1, cur_y, field) && isAvailable(cur_x, cur_y + dy, field) && camefrom(next_index) == -1
            delta = [delta; -1, dy];
        end
    end
end

