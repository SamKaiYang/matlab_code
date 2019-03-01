function G=pos(Ang,Data)
k=(Data(2,3)/2)/sind(60)-Data(2,1);

K=(Data(1,3)/2)*tan(pi/6)-Data(1,1);
A=[0,-K,Data(1,2)];
B=[K*cosd(150),K*sind(150),Data(1,2)];
C=[K*cosd(30),K*sind(30),Data(1,2)];

Ta=[A(1)+0,A(2)-Data(1,4)*cosd(Ang(1)),A(3)+Data(1,4)*sind(Ang(1))];
Tb=[B(1)-Data(1,4)*cosd(Ang(2))*cosd(30),B(2)+Data(1,4)*cosd(Ang(2))*sind(30),B(3)+Data(1,4)*sind(Ang(2))];
Tc=[C(1)+Data(1,4)*cosd(Ang(3))*cosd(30),C(2)+Data(1,4)*cosd(Ang(3))*sind(30),C(3)+Data(1,4)*sind(Ang(3))];

landa_A=(Data(2,4))^2-Ta(1)^2-Ta(2)^2-Ta(3)^2;
landa_B=(Data(2,4))^2-Tb(1)^2-Tb(2)^2-Tb(3)^2;
landa_C=(Data(2,4))^2-Tc(1)^2-Tc(2)^2-Tc(3)^2;

f_A=landa_A-(k*sind(60))^2-(k^2)*9/4+2*Ta(1)*k*sind(60)-3*Ta(2)*k;
f_B=landa_B;
f_C=landa_C-4*(k*sind(60))^2+4*Tc(1)*k*sind(60);

alpha(1)=2*k*sind(60)-2*Ta(1);
alpha(2)=(-3)*k-2*Ta(2);
alpha(3)=-2*Ta(3);
beta(1)=-2*Tb(1);
beta(2)=-2*Tb(2);
beta(3)=-2*Tb(3);
gama(1)=4*k*sind(60)-2*Tc(1);
gama(2)=-2*Tc(2);
gama(3)=-2*Tc(3);

u1=( (alpha(3)-gama(3))*(alpha(2)-beta(2))-(alpha(3)-beta(3))*(alpha(2)-gama(2)) ) / ( (alpha(1)-beta(1))*(alpha(2)-gama(2))-(alpha(1)-gama(1))*(alpha(2)-beta(2)) );
u2=( (alpha(3)-gama(3))*(alpha(1)-beta(1))-(alpha(1)-gama(1))*(alpha(3)-beta(3)) ) / ( (alpha(2)-beta(2))*(alpha(1)-gama(1))-(alpha(1)-beta(1))*(alpha(2)-gama(2)) );
s1=((alpha(2)-gama(2))*(f_A-f_B)-(alpha(2)-beta(2))*(f_A-f_C))/((alpha(1)-beta(1))*(alpha(2)-gama(2))-(alpha(1)-gama(1))*(alpha(2)-beta(2)));
s2=((alpha(1)-gama(1))*(f_A-f_B)-(alpha(1)-beta(1))*(f_A-f_C))/((alpha(2)-beta(2))*(alpha(1)-gama(1))-(alpha(1)-beta(1))*(alpha(2)-gama(2)));

Abar=1+u1^2+u2^2;
Bbar=2*u1*s1+2*u2*s2+alpha(1)*u1+alpha(2)*u2+alpha(3);
Cbar=s1^2+s2^2+alpha(1)*s1+alpha(2)*s2-f_A;
B(3)=(-Bbar-((Bbar^2)-4*Abar*Cbar)^(1/2))/(2*Abar);
if(B(3)>-50)
    B(3)=(-Bbar+((Bbar^2)-4*Abar*Cbar)^(1/2))/(2*Abar);
end
B(1)=u1*B(3)+s1;
B(2)=u2*B(3)+s2;

C=[B(1)+2*k*sind(60),B(2),B(3)];
A=[B(1)+k*sind(60),B(2)-3*k/2,B(3)];
G=[B(1)+k*sind(60),B(2)-k/2,B(3)-Data(2,2)];


end
