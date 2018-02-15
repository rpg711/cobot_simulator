%clear;
clc;

floor2 = 1;
floorref = 5;

mapref = dlmread(sprintf('GHC%d/GHC%d_vector.txt',floorref,floorref));
mapName2 = sprintf('GHC%d',floor2);

%{
m2 = dlmread(strcat(mapName2, '/', mapName2,'.txt'));
m2(:,[2,4]) = -m2(:,[2,4]);
m2 = 0.0859437*(m2-repmat([375.0 655.5],length(m2),2));
m2 = scale*m2 + repmat(shift,length(m2),2);
%}

figure(1);
clf;
hold on;

for i=1:length(mapref)
    line(mapref(i,[1,3]),mapref(i,[2,4]),'color','b');
end

%{
for i=1:length(m2)
    line(m2(i,[1,3]),m2(i,[2,4]),'color','r');
end
%}
hold off;

axis equal;