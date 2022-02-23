clear all;close all;clc;
goal = 10^-3;
ds = .001; %     .00025
rotdia = .38;
rotpch = .18;
rotthk = .05;
vehm = 900;
tiredia = .656;
vel1 = 80;
vel2 = 22.7;
dist = 100;
omeg = 304.9;
padh = .05;
ener = .5*vehm*vel1^2-.5*vehm*vel2^2; %J
qin = ener/2;         %J HALF ACCOUNTED FOR HERE (Due to 2 brake pads)
qin = qin / 4;        %ENERGY PER WHEEL
time = 1.95;        %s 1.95
perc = .1; %Percent time on brakes (Sort of a Duty Cycle for now)
qdotin = (qin/time)*perc; %J/s
hcon = 100;         %W/M^2*k
kay = 28;           %W/m*K
emiscon = .85;         %Emissivity
tinf = 300;         %K
sizey = (rotdia/2-rotpch/2)/ds;
sizex = (rotthk/2)/ds;
relcoef = (cos(pi/sizex)+cos(pi/sizey))^2;
rts = roots([relcoef -16 16]);
rel = min(rts);
t = ones(sizey,sizex) * 1000; %Initial temperature distribution guess
resid(1) = 100;
iter = 1;
sig = emiscon*5.67*10^-8; %Emissivity times Boltsmans Constant
while resid(iter) > goal
    torig = t;
    y=1;x=sizex;               %11 START AT TOP OUTSIDE EDGE WITH HEAT
    t(y,x) = (1-rel)*t(y,x)+rel*(kay*padh*(t(y,x-1)+t(y+1,x))+2*hcon*ds*padh*tinf+ds*(qdotin-2*sig*padh*t(y,x)^4))/(2*padh*(hcon*ds+kay));
    for y = 2:(padh/ds)-1 %10 DO MIDDLE Y OF OUTSIDE EDGE WITH HEAT
        t(y,x) = (1-rel)*t(y,x)+rel*(kay*padh*(t(y+1,x)+t(y-1,x)+2*t(y,x-1))+2*hcon*ds*padh*tinf+2*ds*(qdotin-padh*sig*t(y,x)^4))/(2*padh*(hcon*ds+2*kay));
    end
    y = (padh/ds); %9 END OF HEATED AREA
    t(y,x) = (1-rel)*t(y,x)+rel*(kay*padh*(t(y+1,x)+t(y-1,x)+2*t(y,x-1))+2*hcon*ds*padh*tinf+ds*(qdotin-2*padh*sig*t(y,x)^4))/(2*padh*(hcon*ds+2*kay));
    for y = (padh/ds)+1:sizey-1 %8 DO MIDDLE Y OF OUTSIDE EDGE
        t(y,x) = (1-rel)*t(y,x)+rel*(kay*(t(y+1,x)+t(y-1,x)+2*t(y,x-1))+2*hcon*ds*tinf-2*ds*sig*t(y,x)^4)/(2*(hcon*ds+2*kay));
    end
    y = sizey; %7 BOTTOM CORNER
    t(y,x) = (1-rel)*t(y,x)+rel*(kay*(t(y-1,x)+t(y,x-1))+2*hcon*ds*tinf-2*ds*sig*t(y,x)^4)/(2*(hcon*ds+kay));
    for i = 1:sizex-2 %MID X's
        y=1;x = sizex-i; %2 MID X TOP EDGE
        t(y,x) = (1-rel)*t(y,x)+rel*(kay*(t(y+1,x)+.5*t(y,x+1)+.5*t(y,x-1))+hcon*ds*tinf-ds*(sig*t(y,x)^4))/(hcon*ds+2*kay);
        for y = 2:sizey-1 %5 ABSOLUTE MIDDLES
            t(y,x) = (1-rel)*t(y,x)+rel*(t(y,x+1)+t(y,x-1)+t(y+1,x)+t(y-1,x))/4;
        end
        y = sizey; %6 BOTTOM MIDDLE EDGE
        t(y,x) = (1-rel)*t(y,x)+rel*(kay*(2*t(y-1,x)+t(y,x+1)+t(y,x-1))+2*hcon*ds*tinf-2*ds*sig*t(y,x)^4)/(2*(hcon*ds+2*kay));
    end
    x=1;y=1; %1 TOP LEFT CORNER
    t(y,x) = (1-rel)*t(y,x)+rel*(kay*(t(y,x+1)+t(y+1,x))+hcon*tinf*ds-ds*(sig*t(y,x)^4))/(hcon*ds+2*kay);
    for y = 2:sizey-1 %3 INSULATED EDGE MIDDLE
        t(y,x) = (1-rel)*t(y,x)+rel*(2*t(y,x+1)+t(y+1,x)+t(y-1,x))/4;
    end
    y = sizey;
    t(y,x) = (1-rel)*t(y,x)+rel*(kay*(t(y,x+1)+t(y-1,x))+hcon*tinf*ds-ds*(sig*t(y,x)^4))/(hcon*ds+2*kay);
    iter = iter + 1;
    resid(iter) = sum(sum((torig-t).^2))/((ds-2)*(ds-2));
end
%values = linspace(min(min(t)),max(max(t)),50)
%contour(t,values)                    %Plot Isotherms
colormap(jet(100))
imagesc(t, [min(min(t)) max(max(t))])
axis equal

y=1;x=1;
qu = 0;
qu = qu + (t(y,x)-tinf)/(1/(hcon*.5*ds))+.5*ds*sig*t(y,x)^4;
for x = 2:sizex
    qu = qu + (t(y,x)-tinf)/(1/(hcon*ds))+ds*(sig*t(y,x)^4);
end
for y=2:sizey
    qu = qu + (t(y,x)-tinf)/(1/(hcon*ds))+ds*(sig*t(y,x)^4);
end
for x = 2:sizex-1
    qu = qu + (t(y,x)-tinf)/(1/(hcon*ds))+ds*(sig*t(y,x)^4);
end
x=1;
qu = qu + (t(y,x)-tinf)/(1/(hcon*.5*ds))+.5*ds*sig*t(y,x)^4;
qdotin = qdotin;
qu = qu;
error = (qdotin - qu)/qdotin
highestemp = max(max(t))
iter = iter

%{
Original Writer:    Keith A Young
Date:               03/24/2012
Latest Update:      03/26/2012
Website:            http://racingtech.wordpress.com/
Email:              keith dot a dot young at comcast dot net


This code may only be used under the following conditions:

Non profit use - This code may be used or modified by anyone for non profit
use only if credit is given to the original writer for any code updates,
modifications, or results. Any release of code originating from this must
give its users the same terms as listed in this note, and most include this
note.

For profit - Permission must be granted. For permission, contact keith dot
a dot young at comcast dot net.

Please do not remove this note. If you prefer, it is easiest to move it to
the bottom of the code.

All other rights are held by the original writer, Keith A Young.
%}
%All units are in Metric