function main()
clc;
clear;
figure(1)
clf;
Data=[30 25 400 100;
      0 25 100 250];
%control area
%����
Perspective=[60,30];
mode=0;
%------------------------------------



if(mode==0)
%XYZ=[0 0 -300];%mm
%path(XYZ)
%Ang=neg(XYZ,Perspective,Data);
Ang=[9,9,9]
G=pos(Ang,Data)


end
if(mode==1)
    for i=1:90;
        clf;
        XYZ=[80*sind(4*i),80*cosd(4*i),-200];
        Ang=neg(XYZ,Perspective,Data);
        pos(Ang,Data);
        pause(0.1);
    end
end
end
% function Data=prepare()
% %Data=[RD RH R L;
% %      rd rh r l];
% Data=[30 25 400 100;
%       0 25 100 250];
% end