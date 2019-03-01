function Ang=neg(XYZ,Perspective,Data)
move=0;
K=(Data(1,3)/2)*tan(pi/6)-Data(1,1);
A=[0,-K,Data(1,2)];
B=[K*cosd(150),K*sind(150),Data(1,2)];
C=[K*cosd(30),K*sind(30),Data(1,2)];
k=(Data(2,3)/2)/sind(60)-Data(2,1);
a=[0+XYZ(1),-k+XYZ(2),Data(2,2)+XYZ(3)];
b=[k*cosd(150)+XYZ(1),k*sind(150)+XYZ(2),Data(2,2)+XYZ(3)];
c=[k*cosd(30)+XYZ(1),k*sind(30)+XYZ(2),Data(2,2)+XYZ(3)];
TempA=[A(1)-a(1) A(2)-a(2) A(3)-a(3)];
TempB=[B(1)-b(1) B(2)-b(2) B(3)-b(3)];
TempC=[C(1)-c(1) C(2)-c(2) C(3)-c(3)];
ThitaA=asin((Data(2,4)^2-Data(1,4)^2-TempA(1)^2-TempA(2)^2-TempA(3)^2)/(2*Data(1,4)*(TempA(2)^2+TempA(3)^2)^(1/2)))-atan2(-TempA(2),TempA(3));
temp_B=sin(pi/6+atan2(-TempB(1),TempB(2)));
temp_C=sin(pi/6+atan2(TempC(1),TempC(2)));
ThitaB=asin((Data(2,4)^2-Data(1,4)^2-TempB(1)^2-TempB(2)^2-TempB(3)^2)/(2*Data(1,4)*(TempB(1)^2+TempB(2)^2+TempB(3)^2+(temp_B)^2)^(1/2)))-atan2((TempB(1)^2+TempB(2)^2)^(1/2)*temp_B,TempB(3));
ThitaC=asin((Data(2,4)^2-Data(1,4)^2-TempC(1)^2-TempC(2)^2-TempC(3)^2)/(2*Data(1,4)*(TempC(1)^2+TempC(2)^2+TempC(3)^2+(temp_C)^2)^(1/2)))-atan2((TempC(1)^2+TempC(2)^2)^(1/2)*temp_C,TempC(3));
ThitaA=ThitaA/pi*180;
ThitaB=ThitaB/pi*180;
ThitaC=ThitaC/pi*180;
% ThitaA=roundn(ThitaA,2);
% ThitaB=roundn(ThitaB,2);
% ThitaC=roundn(ThitaC,2);
XYZ
if(move==1)
Draw(A,B,C,a,b,c,ThitaA,ThitaB,ThitaC,Perspective,Data);
end
Ang=[ThitaA,ThitaB,ThitaC]
end
function Draw(A,B,C,a,b,c,ThitaA,ThitaB,ThitaC,Perspective,Data)
% Data=prepare();
Ta=[0+A(1),-Data(1,4)*cosd(ThitaA)+A(2),Data(1,4)*sind(ThitaA)+A(3)];
Tb=[-Data(1,4)*cosd(ThitaB)*cosd(30)+B(1),Data(1,4)*cosd(ThitaB)*sind(30)+B(2),Data(1,4)*sind(ThitaB)+B(3)];
Tc=[Data(1,4)*cosd(ThitaC)*cosd(30)+C(1),Data(1,4)*cosd(ThitaC)*sind(30)+C(2),Data(1,4)*sind(ThitaC)+C(3)];
hold on
grid on
plot3([A(1);Ta(1)],[A(2);Ta(2)],[A(3);Ta(3)],'-r');
plot3([a(1);Ta(1)],[a(2);Ta(2)],[a(3);Ta(3)],'-r');

plot3([B(1);Tb(1)],[B(2);Tb(2)],[B(3);Tb(3)],'-g');
plot3([b(1);Tb(1)],[b(2);Tb(2)],[b(3);Tb(3)],'-g');

plot3([C(1);Tc(1)],[C(2);Tc(2)],[C(3);Tc(3)],'-c');
plot3([c(1);Tc(1)],[c(2);Tc(2)],[c(3);Tc(3)],'-c');

plot3([a(1);b(1)],[a(2);b(2)],[a(3);b(3)],'-b');
plot3([c(1);b(1)],[c(2);b(2)],[c(3);b(3)],'-b');
plot3([a(1);c(1)],[a(2);c(2)],[a(3);c(3)],'-b');
axis([-200,200,-200,200,-300,50]);
%axis manual
view(Perspective(1),Perspective(2))
end
