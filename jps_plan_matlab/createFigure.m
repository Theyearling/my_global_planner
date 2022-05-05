function axishandle = createFigure(field, start, goal)
    % ���if..else�ṹ���������ж����û�д򿪵�figureͼ������������ô���һ��figureͼ
      if isempty(gcbf)                                       %gcbf�ǵ�ǰ����ͼ��ľ����isempty(gcbf)����gcbfΪ�յĻ������ص�ֵ��1������gcbfΪ�ǿյĻ������ص�ֵ��0
      figure('Position',[460 65 700 700], 'MenuBar','none');  %�Դ�����figureͼ��������ã������������Ļ���ľ���Ϊ450��������Ļ�·��ľ���Ϊ50�����ȺͿ�ȶ�Ϊ700�����ҹر�ͼ��Ĳ˵���
      axes('position', [0.01 0.01 0.99 0.99]);               %�����������λ�ã����½ǵ�������Ϊ0.01,0.01   ���Ͻǵ�������Ϊ0.99 0.99  ��������Ϊfigureͼ�����½�����Ϊ0 0   �����Ͻ�����Ϊ1 1 ��
      else
      gcf; cla;   %gcf ���ص�ǰ Figure ����ľ��ֵ��Ȼ������cla����������
      end
      
      n = length(field);  %��ȡ����ĳ��ȣ�����ֵ������n
      %field(field < Inf) = 0; %��fieid�����е��������Ҳ����û���ϰ����λ�ô�����Ϊ0
      pcolor(1:n+1,1:n+1,[field field(:,end); field(end,:) field(end,end)]);%�����һ���ظ��ģ���n X n��Ϊ n+1 X n+1 ��
 
      cmap = flipud(colormap('jet'));  %���ɵ�cmap��һ��256X3�ľ���ÿһ�е�3��ֵ��Ϊ0-1֮�������ֱ������ɫ��ɵ�rgbֵ
      cmap(1,:) = zeros(3,1); cmap(end,:) = ones(3,1); %������cmap�ĵ�һ����Ϊ0 �����һ����Ϊ1
      colormap(flipud(cmap)); %������ɫ�ĵ�ת 
      hold on;
      axishandle = pcolor([1:n+1],[1:n+1],[field field(:,end); field(end,:) field(end,end)]);  %������costchart������չ����ֵ��ɫ�󸳸�axishandle
      [goalposy,goalposx] = ind2sub([n,n],goal);
      [startposy,startposx] = ind2sub([n,n],start);
       plot(goalposx+0.5,goalposy+0.5,'ys','MarkerSize',10,'LineWidth',6);
       plot(startposx+0.5,startposy+0.5,'go','MarkerSize',10,'LineWidth',6);
       %uicontrol('Style','pushbutton','String','RE-DO', 'FontSize',12, 'Position', [1 1 60 40], 'Callback','astardemo');
end

