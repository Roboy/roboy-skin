
body{1}=[140,500;150,500;150,150;140,150;140,500];
body{2}=[500,500;510,500;510,150;500,150;500,500];
% body{3}=[150,500;150,510;500,510;500,500;150,500];

%connect the blocks clockwise
range=[150,500;500,500;500,150;150,150;150,500];
%define the frame range
LED_num = 7;
LED_prec = 5;
LED_range = 120;
lightraynum = ceil(LED_range/LED_prec+1);
LED_dis = (500-150)/(LED_num+1);

nbody=length(body); %number of blocks
lightsource=cell(1,LED_num*lightraynum);
for jj=1:LED_num
    for ii=1:lightraynum
        lightsource{ii+(jj-1)*25}=[150+(jj)*LED_dis,150,180-LED_prec*ii-20];%光源位置和光线方向与x轴正方向夹角
    end
end
nls=length(lightsource); %number of light source
num=1;%number of reflection
lightline=countline(lightsource,body,range,num);%calculate the light ray
figure();
clf;
plot(range(:,1),range(:,2),'k','linewidth',2);%draw the frame
hold on;
for ii=1:nbody
    fill(body{ii}(:,1),body{ii}(:,2),'y','edgecolor','k');%draw the blocks
end
for ii=1:nls
   plot(lightline{ii}(:,1),lightline{ii}(:,2),'b');%draw the light rays
end
hold off;
grid on;