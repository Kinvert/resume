clear all; close all; clc;

m = imread('Rear_Wing_Main_B.bmp');                           %Read Images
BWm = im2bw(m, .5);BWm = bwmorph(BWm,'fill');
BWm = bwmorph(BWm,'clean');BWm = bwperim(BWm);
BWm = bwmorph(BWm,'clean');BWm = bwmorph(BWm,'spur');
[indy,indx] = find(BWm);
main(:,1) = indx-max(indx)-10;
main(:,2) = indy-max(indy);
main = main * [-1,0;0,1];
f = imread('Rear_Wing_Flap_B.bmp');
BWf = im2bw(f, .5);BWf = bwmorph(BWf,'fill');
BWf = bwmorph(BWf,'clean');BWf = bwperim(BWf);
BWf = bwmorph(BWf,'clean');BWf = bwmorph(BWf,'spur');
[infy,infx] = find(BWf);
flap(:,1) = infx-max(indx)-10;
flap(:,2) = infy-max(indy);
flap = flap * [-1,0;0,1];
plot(main(:,1),main(:,2),'b.');
axis([min(main(:,1))  max(main(:,1)) min(main(:,2)) max(main(:,2))]);

[x,y] = ginput(2)                       %MAIN NOSE
rot = atan((y(2)-y(1))/(x(2)-x(1)))+pi/2;
main = main * [cos(rot),-sin(rot);sin(rot),cos(rot)];
flap = flap * [cos(rot),-sin(rot);sin(rot),cos(rot)];
plot(main(:,1),main(:,2),'b.');
[x,y] = ginput(1)
edit = find(main(:,2) > y(1));
for i = 1:length(edit)
    mainb(i,:) = main(edit(i),:);
end
mainb = unique(mainb, 'rows');
mainb = sortrows(mainb);
mainc(:,2) = smooth(mainb(:,1),mainb(:,2),0.2,'rloess');mainc(:,1)=mainb(:,1);
plot(mainb(:,1),mainb(:,2),'r.');hold on;
plot(mainc(:,1),mainc(:,2),'g.');axis equal;hold off
[x,y] = ginput(1)                       %OVERWRITE MAIN WITH SMOOTH NOSE
edit = find(main(:,2) > y(1));
for i = 1:length(edit)
    main(edit(i),:) = [];
    edit = edit - 1;
end
edit = find(mainc(:,2) > y(1));
for i = 1:length(edit)
    main(length(main)+1,:) = mainc(edit(i),:);
end
rot = -rot;
main = main * [cos(rot),-sin(rot);sin(rot),cos(rot)];
flap = flap * [cos(rot),-sin(rot);sin(rot),cos(rot)];

plot(flap(:,1),flap(:,2),'b.');axis equal;         %FLAP NOSE
[x,y] = ginput(2)                        
rot = atan((y(2)-y(1))/(x(2)-x(1)))+pi/2;
main = main * [cos(rot),-sin(rot);sin(rot),cos(rot)];
flap = flap * [cos(rot),-sin(rot);sin(rot),cos(rot)];
plot(flap(:,1),flap(:,2),'b.');
[x,y] = ginput(1)
edit = find(flap(:,2) > y(1));
for i = 1:length(edit)
    flapb(i,:) = flap(edit(i),:);
end
flapb = unique(flapb, 'rows');
flapb = sortrows(flapb);
flapc(:,2) = smooth(flapb(:,1),flapb(:,2),0.2,'rloess');flapc(:,1)=flapb(:,1);
plot(flapb(:,1),flapb(:,2),'r.');hold on;
plot(flapc(:,1),flapc(:,2),'g.');axis equal;hold off
[x,y] = ginput(1)                       %OVERWRITE FLAP WITH SMOOTH NOSE
edit = find(flap(:,2) > y(1));
for i = 1:length(edit)
    flap(edit(i),:) = [];
    edit = edit - 1;
end
edit = find(flapc(:,2) > y(1));
for i = 1:length(edit)
    flap(length(flap)+1,:) = flapc(edit(i),:);
end
rot = -rot;
main = main * [cos(rot),-sin(rot);sin(rot),cos(rot)];
flap = flap * [cos(rot),-sin(rot);sin(rot),cos(rot)];     
%END OF BOTH INITIAL NOSE ROUNDING

plot(main(:,1),main(:,2),'r.');                        %BEGIN PICKING MAIN TAIL
[x,y] = ginput(1)
plot(main(:,1),main(:,2),'b.');axis square;axis([x-100 x+100 y-100 y+100]);
[x,y] = ginput(1)
compare = y;
while y > compare-5                                             %DELETE OR CONTINUE
    plot(main(:,1),main(:,2),'b.');axis square;axis([x-15 x+15 y-15 y+15]);
    [x,y] = ginput(1)
    thing1 = delaunay(main(:,1),main(:,2));
    k1 = dsearchn(main,thing1,[x,y]);
    main(k1,:) = [];
end
for i=1:2                                                       %Pick 2 to make tail point
    [x,y] = ginput(1)
    thing1 = delaunay(main(:,1),main(:,2));
    k1 = dsearchn(main,thing1,[x,y]);
    pts(i,:) = main(k1,:);
end
tail = [mean(pts(:,1)) mean(pts(:,2))];
main(length(main)+1,:) = tail;                                   %CREATE TAIL POINT
main(:,1) = main(:,1) - tail(1);main(:,2) = main(:,2) - tail(2); %TAIL 0 0
flap(:,1) = flap(:,1) - tail(1);flap(:,2) = flap(:,2) - tail(2);
plot(main(:,1),main(:,2),'g.');
nose = ginput(1)                                  %PICK APPROX NOSE
%rot = atan((nose(2)-tail(2))/(nose(1)-tail(1)))+pi;
rot = atan((nose(2))/(nose(1)))+pi;
main = main * [cos(rot),-sin(rot);sin(rot),cos(rot)];
flap = flap * [cos(rot),-sin(rot);sin(rot),cos(rot)];
for i=1:10
    test = min(main(:,1));
    edit = find(main(:,1)==test);
    nose = main(edit,:);
    rot = atan((nose(2))/(nose(1)));
    main = main * [cos(rot),-sin(rot);sin(rot),cos(rot)];
    flap = flap * [cos(rot),-sin(rot);sin(rot),cos(rot)];
end
test = min(main(:,1));
main = main * [1/abs(test),0;0,1/abs(test)];
flap = flap * [1/abs(test),0;0,1/abs(test)];
main(:,1) = main(:,1) + 1;
flap(:,1) = flap(:,1) + 1;

mainb = main;                                       %SEPARATE TOP BOTTOM MAIN 
plot(main(:,1),main(:,2),'r.');axis equal;hold on
[x,y] = ginput(10); mid = polyfit(x,y,4);
less = find(main(:,1) < x(1));
for i = 1:length(less)
    if mainb(less(i),2) > 0
        valuet(i,:) = mainb(less(i),:);
    end
    if mainb(less(i),2) < 0
        valueb(i,:) = mainb(less(i),:);
    end
    mainb(less(i),:) = [];
    less = less - 1;
end
valuet = unique(valuet, 'rows');valueb = unique(valueb, 'rows');
topx = find(mainb(:,2) > mid(1).*mainb(:,1).^4+mid(2).*mainb(:,1).^3+mid(3).*mainb(:,1).^2+mid(4).*mainb(:,1)+mid(5));
botx = find(mainb(:,2) < mid(1).*mainb(:,1).^4+mid(2).*mainb(:,1).^3+mid(3).*mainb(:,1).^2+mid(4).*mainb(:,1)+mid(5));
for i = 1:length(topx)
    valuet(length(valuet)+1,:) = mainb(topx(i),:);
end
for i = 1:length(botx)
    valueb(length(valueb)+1,:) = mainb(botx(i),:);
end
plot(valuet(:,1),valuet(:,2),'r.');axis([-.1 1.1 -1 1]);axis equal;hold on
plot(valueb(:,1),valueb(:,2),'b.')
valuet = sortrows(valuet);valueb = sortrows(valueb);mainb = sortrows(mainb);
valuet = unique(valuet, 'rows');valueb = unique(valueb, 'rows');

flapb = flap;                                            %SEPARATE TOP BOTTOM FLAP
test = min(flapb(:,1));
vert = find(flapb(:,1)==test);
vert = flapb(vert,2);
flapb(:,2) = flapb(:,2) - vert;
hold off;plot(flapb(:,1),flapb(:,2),'r.');axis equal;
[x,y] = ginput(10); mid = polyfit(x,y,4);
less = find(flap(:,1) < x(1));
for i = 1:length(less)
    if flapb(less(i),2) > 0
            valuetf(i,:) = flapb(less(i),:);
    end
    if flapb(less(i),2) < 0
        valuebf(i,:) = flapb(less(i),:);
    end
    flapb(less(i),:) = [];
    less = less - 1;
end
valuetf = unique(valuetf, 'rows');valuebf = unique(valuebf, 'rows');
topxf = find(flapb(:,2) > mid(1).*flapb(:,1).^4+mid(2).*flapb(:,1).^3+mid(3).*flapb(:,1).^2+mid(4).*flapb(:,1)+mid(5));
botxf = find(flapb(:,2) < mid(1).*flapb(:,1).^4+mid(2).*flapb(:,1).^3+mid(3).*flapb(:,1).^2+mid(4).*flapb(:,1)+mid(5));
for i = 1:length(topxf)
    valuetf(length(valuetf)+1,:) = flapb(topxf(i),:);
end
for i = 1:length(botxf)
    valuebf(length(valuebf)+1,:) = flapb(botxf(i),:);
end
valuetf(:,2) = valuetf(:,2) + vert;valuebf(:,2) = valuebf(:,2) + vert;
plot(valuetf(:,1),valuetf(:,2),'r.');axis([-.1 1.1 -1 1]);axis equal;hold on
plot(valuebf(:,1),valuebf(:,2),'b.');
valuetf = sortrows(valuetf);valuebf = sortrows(valuebf);flapb = sortrows(flapb);
valuetf = unique(valuetf, 'rows');valuebf = unique(valuebf, 'rows');

clear mainz;clear mainb
valueb = flipdim(valueb,1);
for i=1:length(valueb(:,1))
    mainz(i,:) = valueb(i,:);
end
for i = 1:length(valuet(:,1))
    mainz(i+length(valueb(:,1)),:) = valuet(i,:);
end
clear flapz;clear flapb
valuebf = flipdim(valuebf,1);
for i=1:length(valuebf(:,1))
    flapz(i,:) = valuebf(i,:);
end
for i = 1:length(valuetf(:,1))
    flapz(i+length(valuebf(:,1)),:) = valuetf(i,:);
end

mainb(:,1) = smooth(mainz(:,1),.005,'rloess');
mainb(:,2) = smooth(mainz(:,2),.005,'rloess');
mainc = mainz;
x = 1;j = 0;
while x(1) > 0
    j = j + 1;
    mainb = mainc;
    mainb(:,2) = smooth(mainb(:,2),.01*j,'rloess');
    plot(mainz(:,1),mainz(:,2),'r.');hold on;
    plot(mainc(:,1),mainc(:,2),'b.')
    plot(mainb(:,1),mainb(:,2),'g.');axis equal;hold off
    pause
    [x,y] = ginput(2)                       
    edit = find(mainc(:,1) > x(1,1));
    for i = 1:length(edit)
        if mainc(edit(i),1) < x(2,1)
            mainc(edit(i),:) = mainb(edit(i),:);
        end
    end
end

mess = find(flapz(:,1)==0);
flapz(mess,:) = [];
flapb(:,1) = smooth(flapz(:,1),.005,'rloess');
flapb(:,2) = smooth(flapz(:,2),.005,'rloess');
flapc = flapz;
x = 1;j = 0;
while x(1) > 0
    j = j + 1;
    flapb = flapc;
    flapb(:,2) = smooth(flapb(:,2),.01*j,'rloess');
    plot(flapz(:,1),flapz(:,2),'r.');hold on;
    plot(flapc(:,1),flapc(:,2),'b.')
    plot(flapb(:,1),flapb(:,2),'g.');axis equal;hold off
    pause
    [x,y] = ginput(2)                       
    edit = find(flapc(:,1) > x(1,1));
    for i = 1:length(edit)
        if flapc(edit(i),1) < x(2,1)
            flapc(edit(i),:) = flapb(edit(i),:);
        end
    end
end

%------------------------------------------------------------------------------------------------------
clear point
point(1,:) = mainc(1,:);i=1;k=1;
a = .196;b = .001;check = 0;
while check == 0
    for j=2:250
        sx = -a*mainc(k,1)^2+a*mainc(k,1)+b;
        s = 0;
        while s < sx && i < length(mainc) - 1
            i = i + 1;
            s = s + sqrt((mainc(i,1)-mainc(i+1,1))^2+(mainc(i,2)-mainc(i+1,2))^2);
        end
        k = i;
        point(j,:) = mainc(k,:);
    end
    if point(200,1)~=point(201,1) && point(200,1)~=point(201,1)
        check = 5;
    end
    b = b * .875;
    a = a * .875;
    i = 1;k = 1;
end
i = 1;
while point(length(point),1)==point(length(point)-1,1) && point(length(point)-1,1)==point(length(point),1)
    point(length(point),:) = [];
    i = i + 1;
end

clear pointf
tran = min(flapc(:,1));
flapc(:,1) = flapc(:,1) - tran;
pointf(1,:) = flapc(1,:);i=1;k=1;
a = -5;b = 2.5;check = 0;
while check == 0
    for j=2:250
        sxf = -a*flapc(k,1)^2+b*flapc(k,1)+.001;
        sf = 0;
        while sf < sxf && i < length(flapc) - 1
            i = i + 1;
            sf = sf + sqrt((flapc(i,1)-flapc(i+1,1))^2+(flapc(i,2)-flapc(i+1,2))^2);
        end
        k = i;
        pointf(j,:) = flapc(k,:);
    end
    if pointf(200,1)~=pointf(201,1) && pointf(200,1)~=pointf(201,1)
        check = 5;
    end
    b = b * .875;
    a = a * .875;
    i = 1;k = 1;
end
i = 1;
while pointf(length(pointf),1)==pointf(length(pointf)-1,1) && pointf(length(pointf)-1,1)==pointf(length(pointf),1)
    pointf(length(pointf),:) = [];
    i = i + 1;
end
flapc(:,1) = flapc(:,1) + tran;
pointf(:,1) = pointf(:,1) + tran;
plot(point(:,1),point(:,2),'b.');hold on
plot(pointf(:,1),pointf(:,2),'b.')
axis equal

point = transpose(point);
pointf = transpose(pointf);
fid = fopen('newair.txt', 'w');
fprintf(fid, 'NEWAIR\r');
fprintf(fid, '%6.5f %6.5f\r', point);
fprintf(fid, '%6.5f %6.5f\r', [9999.9 9999.9]);
fprintf(fid, '%6.5f %6.5f\r', pointf);
fclose(fid);
type newair.txt
clear valueb valuet valuebs valuets valuebs2 valuets2;


%fit
%imshow(BWm)