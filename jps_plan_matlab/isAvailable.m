% �ж������Ƿ�Ϸ����Ƿ����ϰ���
function judge = isAvailable(x, y, field)
    [height, width] = size(field);
    if x < 1 || x > height || y < 1 || y > width || field(x, y) == 1 
        judge = 0;
        return;
    end
    judge = 1;
end

