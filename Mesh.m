Rec_width = 8.2; % cm
Rec_height = 9.1;
LED_size = [0.2 0.2];
OD_size = [0.2 0.2];
hole_num = [6 6 6 6];
dis_to_edge = [0.6 0.6 0.6 0.6]; % cm
shape_mode = 2; % 1:two sides LED; 2:LED and OD combine 3:self define combine
distribute = 1; %
press_pos = [3,4];
press_level = 0;
press_area = 0.5;
view_mode = 1; % 1:down view; 0:3D mode;

[X,Y] = meshgrid(0:0.05:Rec_width,0:0.05:Rec_height);
Rec_size = [Rec_width Rec_height Rec_width Rec_height];
press_level = press_level/10;
loss = press_level*exp(-((X-press_pos(1)).^2+(Y-press_pos(2)).^2)./press_area.^2);
figure; 
set(gcf,'doublebuffer','on'); 

Comp = zeros(sum(hole_num),3);
Comp(1:hole_num(1),1) = dis_to_edge(1):(Rec_size(1)-2*dis_to_edge(1))/(hole_num(1)-1):Rec_size(1)-dis_to_edge(1);
Comp(1:hole_num(1),2) = 0*ones(hole_num(1),1);
Comp(hole_num(1)+1:sum(hole_num(1:2)),1) = Rec_size(1)*ones(hole_num(2),1);
Comp(hole_num(1)+1:sum(hole_num(1:2)),2) = dis_to_edge(2):(Rec_size(2)-2*dis_to_edge(2))/(hole_num(2)-1):Rec_size(2)-dis_to_edge(2);
Comp(sum(hole_num(1:2))+1:sum(hole_num(1:3)),1) = Rec_size(3)-dis_to_edge(3):-(Rec_size(3)-2*dis_to_edge(3))/(hole_num(3)-1):dis_to_edge(3);
Comp(sum(hole_num(1:2))+1:sum(hole_num(1:3)),2) = Rec_size(2)*ones(hole_num(3),1);
Comp(sum(hole_num(1:3))+1:end,1) = 0*ones(hole_num(4),1);
Comp(sum(hole_num(1:3))+1:end,2) = Rec_size(4)-dis_to_edge(4):-(Rec_size(4)-2*dis_to_edge(4))/(hole_num(4)-1):dis_to_edge(4);

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

k = 1; 
j = 1;
while k
    s=get(gcf,'currentkey'); 
    if strcmp(s,'space')
        clc;k=0; 
    end 
    axis([0 Rec_width 0 Rec_height 0 1]);
    if Comp(j,3) == 1
        pause(0.3);
        if (j<=hole_num(1))||((j>sum(hole_num(1:2)))&&(j<=sum(hole_num(1:3))))
            Z = sqrt(cos(atan((X-Comp(j,1))./(Y-Comp(j,2))))) - loss;
            mesh(X,Y,Z);
            if view_mode
                view(0,-90);
            end
            colorbar;
        else
            Z = sqrt(abs(sin(atan((X-Comp(j,1))./(Y-Comp(j,2)))))) - loss;
            mesh(X,Y,Z);
            if view_mode
                view(0,-90);
            end
            colorbar;
        end
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
    hold on;
    if Comp(j,3) == 1
        for i = 1:sum(hole_num)
            if (Comp(i,3) == 0)
                plot([Comp(j,1),Comp(i,1)],[Comp(j,2),Comp(i,2)],'Color','r');
            end
        end
    end
    hold off;
    j = j+1;
    if j > sum(hole_num)
        j = 1;
    end
end