function [sout]=newsource(s,edges)
    x0=s(1);y0=s(2);deg0=s(3);
    x1=edges(:,1);x2=edges(:,3);
    y1=edges(:,2);y2=edges(:,4);
    deg=edges(:,5);
    t=((y0-y1).*(x1-x2)-(x0-x1).*(y1-y2))./...     %intersection point
        (cosd(deg0).*(y1-y2)-sind(deg0).*(x1-x2));
    xn=cosd(deg0).*t+x0;
    yn=sind(deg0).*t+y0;
    mask=t>0&(xn-x1).*(xn-x2)<=0&(yn-y1).*(yn-y2)<=0;
    x=xn(mask);y=yn(mask);deg=deg(mask);  
    d=sqrt((x-x0).^2+(y-y0).^2);          %distance between intersection point and light source
    ind=find(d==min(d));                  %min distance
    if(isempty(ind))
        sout=[nan nan nan];               
    else
        dd=mod(deg(ind(1))-deg0,360);     
        if cosd(dd)>=0
            dout=nan; %no reflection if the angle is less than 90
        else
            dout=2*deg(ind(1))-180-deg0; %calculate reflected light
        end
        sout=[x(ind(1)) y(ind(1)) dout]; 
    end
end