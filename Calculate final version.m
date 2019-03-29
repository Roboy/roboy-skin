Rec_width = 10; % cm
Rec_height = 10;
LED_size = [0.2 0.2];
OD_size = [0.2 0.2];
hole_num = [6 6 6 6];
dis_to_edge = [0.6 0.6 0.6 0.6]; % cm
shape_mode = 2; % 1:two sides LED; 2:LED and OD combine 3:self define combine
distribute = 1; %
press_pos = [8,3];
press_level = 0;
press_area = 1;
material_loss = 0.001;
view_mode = 1; % 1:down view; 0:3D mode;

[X,Y] = meshgrid(0:0.05:Rec_width,0:0.05:Rec_height);
Rec_size = [Rec_width Rec_height Rec_width Rec_height];
press_level = press_level/10;
loss = press_level*exp(-((X-press_pos(1)).^2+(Y-press_pos(2)).^2)./press_area.^2);
figure; 
set(gcf,'doublebuffer','on'); 


Comp = zeros(sum(hole_num),5); % 1,2 for position, 3 for LED or sensor, 4,5 for opposite position
% first edge
Comp(1:hole_num(1),1) = dis_to_edge(1):(Rec_size(1)-2*dis_to_edge(1))/(hole_num(1)-1):Rec_size(1)-dis_to_edge(1);
Comp(1:hole_num(1),2) = 0*ones(hole_num(1),1);
Comp(1:hole_num(1),4) = dis_to_edge(1):(Rec_size(1)-2*dis_to_edge(1))/(hole_num(1)-1):Rec_size(1)-dis_to_edge(1);
Comp(1:hole_num(1),5) = Rec_size(2)*ones(hole_num(1),1);
% second edge
Comp(hole_num(1)+1:sum(hole_num(1:2)),1) = Rec_size(1)*ones(hole_num(2),1);
Comp(hole_num(1)+1:sum(hole_num(1:2)),2) = dis_to_edge(2):(Rec_size(2)-2*dis_to_edge(2))/(hole_num(2)-1):Rec_size(2)-dis_to_edge(2);
Comp(hole_num(1)+1:sum(hole_num(1:2)),4) = 0*ones(hole_num(2),1);
Comp(hole_num(1)+1:sum(hole_num(1:2)),5) = dis_to_edge(2):(Rec_size(2)-2*dis_to_edge(2))/(hole_num(2)-1):Rec_size(2)-dis_to_edge(2);
% third edge
Comp(sum(hole_num(1:2))+1:sum(hole_num(1:3)),1) = Rec_size(3)-dis_to_edge(3):-(Rec_size(3)-2*dis_to_edge(3))/(hole_num(3)-1):dis_to_edge(3);
Comp(sum(hole_num(1:2))+1:sum(hole_num(1:3)),2) = Rec_size(2)*ones(hole_num(3),1);
Comp(sum(hole_num(1:2))+1:sum(hole_num(1:3)),4) = Rec_size(3)-dis_to_edge(3):-(Rec_size(3)-2*dis_to_edge(3))/(hole_num(3)-1):dis_to_edge(3);
Comp(sum(hole_num(1:2))+1:sum(hole_num(1:3)),5) = 0*ones(hole_num(3),1);
% forth edge
Comp(sum(hole_num(1:3))+1:end,1) = 0*ones(hole_num(4),1);
Comp(sum(hole_num(1:3))+1:end,2) = Rec_size(4)-dis_to_edge(4):-(Rec_size(4)-2*dis_to_edge(4))/(hole_num(4)-1):dis_to_edge(4);
Comp(sum(hole_num(1:3))+1:end,4) = Rec_size(3)*ones(hole_num(4),1);
Comp(sum(hole_num(1:3))+1:end,5) = Rec_size(4)-dis_to_edge(4):-(Rec_size(4)-2*dis_to_edge(4))/(hole_num(4)-1):dis_to_edge(4);

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

Intensity = zeros(sum(hole_num));
k = 1; 
j = 1;
while k
    s=get(gcf,'currentkey'); 
    if strcmp(s,'space')
        k=0; 
    end 
    if Comp(j,3) == 1
        pause(0.5);
        clf;
        set(gca,'box','on');
        set(gca,'xtick',[]);
        set(gca,'ytick',[]);
        title('Simulation of Current of Phototransistor');
        xlabel(['Material loss: ',num2str(material_loss),'  Press position: [',num2str(press_pos(1)),',',num2str(press_pos(2)),']','  Press level: ',num2str(press_area)]...
                ,'position',[5,-0.5]);
    end
    % lines
    hold on;
    if Comp(j,3) == 1
        for i = 1:sum(hole_num)
            if (Comp(i,3) == 0)
                % distance to gausian center
                dis_g = abs(det([[Comp(j,1),Comp(j,2)]-[Comp(i,1),Comp(i,2)];press_pos-[Comp(i,1),Comp(i,2)]]))/norm([Comp(j,1),Comp(j,2)]-[Comp(i,1),Comp(i,2)]);
                % distance from LED to sensor
                dis_l = sqrt((Comp(j,1)-Comp(i,1))^2+(Comp(j,2)-Comp(i,2))^2);
                % angle for LED and sensor
                a = [Comp(j,4),Comp(j,5)]-[Comp(j,1),Comp(j,2)];
                b = [Comp(i,1),Comp(i,2)]-[Comp(j,1),Comp(j,2)];
                angle_LED = dot(a,b)/(sqrt(dot(a,a))*sqrt(dot(b,b)));
                a = [Comp(i,4),Comp(i,5)]-[Comp(i,1),Comp(i,2)];
                b = [Comp(j,1),Comp(j,2)]-[Comp(i,1),Comp(i,2)];
                angle_sensor = dot(a,b)/(sqrt(dot(a,a))*sqrt(dot(b,b)));
                % calculate intensity
                intensity1 = pi*dis_l*tan(22.5*pi/180)*10*0.8*sqrt(angle_LED)*sqrt(angle_sensor)*exp(-dis_l*material_loss)*exp(-(1/sqrt(2*pi)/press_area*exp(-press_level*(dis_g/press_area)^2/2)))/(dis_l^2);
                intensity = 1.09976+(0.39792-1.09976)/(1+exp((intensity1-0.7005)/0.16548));
                text(Comp(i,1)-1,Comp(i,2)-0.2,mat2str(intensity));
                Intensity(j,i) = intensity;
%                 if intensity > 1
%                     [angle_LED,angle_sensor,dis_l,exp(-dis_l*material_loss),dis_g/press_area,exp((1/sqrt(2*pi)/press_area*exp(-(dis_g/press_area)^2/2)))]
%                 end
                plot([Comp(j,1),Comp(i,1)],[Comp(j,2),Comp(i,2)],'Color','r');
            end
        end
    end
    ang=0:0.01:2*pi; 
    xp=press_area*cos(ang);
    yp=press_area*sin(ang);
    plot(press_pos(1)+xp,press_pos(2)+yp,'b');
    hold off;
        % sensors and LEDs
    for i = 1:hole_num(1) % 1 for red, means LED
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

    j = j+1;
    if j > sum(hole_num)
        j = 1;
        %k = 0;
    end
end
k = 1;
l = 1;
Result = [];
for i = 1: sum(hole_num)
    if Comp(i,3) == 1
        for j = 1: sum(hole_num)
            if Comp(j,3) == 0
                Result(k,l) = Intensity(i,j);
                l = l+1;
            end
        end
        k = k+1;
        l = 1;
    end
end
Result