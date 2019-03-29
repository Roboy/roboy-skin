Rec_width = 8.2; % cm
Rec_height = 9.1;
Rec_size = [Rec_width Rec_height Rec_width Rec_height];
LED_num = 7;
LED_size = [0.2 0.2];
OD_num = 7;
OD_size = [0.2 0.2];
hole_num = [6 6 6 6];
dis_to_edge = [0.6 0.6 0.6 0.6]; % cm
shape_mode = 2; % 1:two sides LED; 2:LED and OD combine 3:self define combine
distribute = 1; %
color = [1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0;...
         1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0;...
         1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0;...
         1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0;...
         1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0;...
         1 1 0; 1 0 1; 0 1 1; 1 0 0; 0 1 0; 0 0 1; 0 0 0];
figure;
set(gcf,'doublebuffer','on');
rectangle('Position',[0 0 Rec_width Rec_height]);
hold on;

Comp = zeros(sum(hole_num),3);
Comp(1:hole_num(1),1) = dis_to_edge(1):(Rec_size(1)-2*dis_to_edge(1))/(hole_num(1)-1):Rec_size(1)-dis_to_edge(1);
Comp(1:hole_num(1),2) = 0*ones(hole_num(1),1);
Comp(hole_num(1)+1:sum(hole_num(1:2)),1) = Rec_size(1)*ones(hole_num(2),1);
Comp(hole_num(1)+1:sum(hole_num(1:2)),2) = dis_to_edge(2):(Rec_size(2)-2*dis_to_edge(2))/(hole_num(2)-1):Rec_size(2)-dis_to_edge(2);
Comp(sum(hole_num(1:2))+1:sum(hole_num(1:3)),1) = dis_to_edge(3):(Rec_size(3)-2*dis_to_edge(3))/(hole_num(3)-1):Rec_size(3)-dis_to_edge(3);
Comp(sum(hole_num(1:2))+1:sum(hole_num(1:3)),2) = Rec_size(2)*ones(hole_num(3),1);
Comp(sum(hole_num(1:3))+1:end,1) = 0*ones(hole_num(4),1);
Comp(sum(hole_num(1:3))+1:end,2) = dis_to_edge(4):(Rec_size(4)-2*dis_to_edge(4))/(hole_num(4)-1):Rec_size(4)-dis_to_edge(4);

if shape_mode == 1
    Comp(1:hole_num(1),3) = ones(hole_num(1),1);
    Comp(sum(hole_num(1:3))+1:end,3) = ones(hole_num(4),1);    
end

if shape_mode == 2
    i = 1;
    while i<=sum(hole_num)
        Comp(i,3) = 1;
        i=i+2;
    end
end

if shape_mode == 3
    
end

for i = 1:hole_num(1)
    if Comp(i,3) == 1
        rectangle('Position',[Comp(i,1)-LED_size(1)/2 Comp(i,2) LED_size],'FaceColor',[1,0,0]);
    else
        rectangle('Position',[Comp(i,1)-OD_size(1)/2 Comp(i,2) OD_size],'FaceColor',[0,0,1]);
    end
end

for i = hole_num(1)+1:sum(hole_num(1:2))
    if Comp(i,3) == 1
        rectangle('Position',[Comp(i,1)-LED_size(2) Comp(i,2)-LED_size(1)/2 LED_size],'FaceColor',[1,0,0]);
    else
        rectangle('Position',[Comp(i,1)-OD_size(2) Comp(i,2)-OD_size(1)/2 OD_size],'FaceColor',[0,0,1]);
    end
end

for i = sum(hole_num(1:2))+1:sum(hole_num(1:3))
    if Comp(i,3) == 1
        rectangle('Position',[Comp(i,1)-LED_size(1)/2 Comp(i,2)-LED_size(2) LED_size],'FaceColor',[1,0,0]);
    else
        rectangle('Position',[Comp(i,1)-OD_size(1)/2 Comp(i,2)-OD_size(2) OD_size],'FaceColor',[0,0,1]);
    end
end

for i = sum(hole_num(1:3))+1:sum(hole_num)
    if Comp(i,3) == 1
        rectangle('Position',[Comp(i,1) Comp(i,2)-LED_size(2)/2 LED_size],'FaceColor',[1,0,0]);
    else
        rectangle('Position',[Comp(i,1) Comp(i,2)-OD_size(2)/2 OD_size],'FaceColor',[0,0,1]);
    end
end

for i = 1:sum(hole_num)
    if Comp(i,3) == 1
        for j = 1:sum(hole_num)
            if (Comp(j,3) == 0)
                plot([Comp(i,1),Comp(j,1)],[Comp(i,2),Comp(j,2)],'Color',color(i,:));
            end
        end
    end
end

% Pos_LED_x = 0:Rec_width/(LED_num+1):Rec_width;
% Pos_LED_x(1) = [];
% Pos_LED_x(end) = [];
% Pos_LED_y = 0*ones(1,7);
% Pos_OD_x = 0:Rec_width/(LED_num+1):Rec_width;
% Pos_OD_x(1) = [];
% Pos_OD_x(end) = [];
% Pos_OD_y = Rec_width*ones(1,7);
% 
% Pos_LED_yy = 0:Rec_width/(LED_num+1):Rec_width;
% Pos_LED_yy(1) = [];
% Pos_LED_yy(end) = [];
% Pos_LED_xx = 0*ones(1,7);
% Pos_OD_yy = 0:Rec_width/(LED_num+1):Rec_width;
% Pos_OD_yy(1) = [];
% Pos_OD_yy(end) = [];
% Pos_OD_xx = Rec_width*ones(1,7);
% for i=1:LED_num
%     H_LED(i) = rectangle('Position',[Pos_LED_x(i)-0.1 Pos_LED_y(i) 0.2 0.2]);
% end
% for i=1:LED_num
%     H_LED(i) = rectangle('Position',[Pos_LED_xx(i) Pos_LED_yy(i)-0.1 0.2 0.2]);
% end
% for i=1:OD_num
%     H_OD(i) = rectangle('Position',[Pos_OD_x(i)-0.1 Pos_OD_y(i)-0.2 0.2 0.2]);
% end
% for i=1:OD_num
%     H_OD(i) = rectangle('Position',[Pos_OD_xx(i)-0.2 Pos_OD_yy(i)-0.1 0.2 0.2]);
% end
% 
% for i = 1:LED_num
%     for j = 1:OD_num
%         plot([Pos_LED_x(i),Pos_OD_x(j)],[Pos_LED_y(i),Pos_OD_y(j)],'Color',color(i,:));
%         plot([Pos_LED_xx(i),Pos_OD_xx(j)],[Pos_LED_yy(i),Pos_OD_yy(j)],'Color',color(i,:));
%     end
% end
