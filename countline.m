function [lineout]=countline(ls,body,range,num)
body=[body,range];
edges=[];
for ii=1:length(body)
    x1=body{ii}(1:end-1,1);
    x2=body{ii}(2:end,1);
    y1=body{ii}(1:end-1,2);
    y2=body{ii}(2:end,2);
    deg=mod(rad2deg(atan2(y2-y1,x2-x1))+90,360);%calculate angles
    edges=[edges;[x1 y1 x2 y2 deg]];
end 
lineout=cell(size(ls));
for ii=1:length(ls);
    s=ls{ii};         %s is the current light source
    lin=[s(1),s(2)];  
    n=1;
    while(~isnan(s(3))&&n<=num) %not achieve frame and reflect less than reflection number
        s=newsource(s,edges);  %calculate new source
        lin=[lin;[s(1) s(2)]]; %add a point to the light
        n=n+1;  %add reflection times
    end
    lineout{ii}=lin;        
end
end