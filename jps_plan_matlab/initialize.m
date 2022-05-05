function [field, start, goal] = initialize(n,wallpercent)
    field = zeros(n,n);%����һ��n*n�ĵ�λ����+0��10��Χ�ڵ�һ�������
    field(ind2sub([n n],ceil(n^2.*rand(n*n*wallpercent,1)))) = 1;%����ȡ��
    % ���������ʼ�����ֹ��
    start = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));  %���������ʼ�������ֵ
    goal = sub2ind([n,n],ceil(n.*rand),ceil(n.*rand));   %���������ֹ�������ֵ
    field(start) = 0; field(goal) = 0;  %�Ѿ�������ʼ�����ֹ�㴦��ֵ��Ϊ0
end

