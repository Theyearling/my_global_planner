function [field, start, goal] = initialize(n,wallpercent)
    field = zeros(n,n);%生成一个n*n的单位矩阵+0到10范围内的一个随机数
    field(ind2sub([n n],ceil(n^2.*rand(n*n*wallpercent,1)))) = 1;%向上取整
    % 随机生成起始点和终止点
    start = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));  %随机生成起始点的索引值
    goal = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));   %随机生成终止点的索引值
    field(start) = 0; field(goal) = 0;  %把矩阵中起始点和终止点处的值设为0
end

