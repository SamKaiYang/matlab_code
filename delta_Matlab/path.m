function path()
clc;
clear;
figure(1)
clf;
Perspective=[60,30];
Data=[30 25 400 100;
      0 25 100 250];
  %粉色虛線是點對點拉直線
  %紅色實線是速度規劃前
  %綠色實線是速度規劃後
  %沿著X軸、Z軸移動會有問題
pre=[0,0,-200]
next=[-100 100 -200];
G = pre



[A,B,C,a,b,c,pre_Ang]=neg(pre,Perspective,Data);
[A1,B1,C1,a1,b1,c1,next_Ang]=neg(next,Perspective,Data);

hold on
Draw(A,B,C,a,b,c,pre_Ang(1),pre_Ang(2),pre_Ang(3),Perspective,Data,'-k');
plot3(pre(:,1),pre(:,2),pre(:,3),'bo-','linewidth',3,'markeredgecolor','r','markerfacecolor','r','markersize',10);
%Draw(A1,B1,C1,a1,b1,c1,next_Ang(1),next_Ang(2),next_Ang(3),Perspective,Data,'-b');
%plot3(next(:,1),next(:,2),next(:,3),'bo-','linewidth',3,'markeredgecolor','r','markerfacecolor','r','markersize',10);
aa=pos(pre_Ang,Data);
bb=pos(next_Ang,Data);
%plot3([aa(1);bb(1)],[aa(2);bb(2)],[aa(3);bb(3)],':m');

    AAng=(next_Ang(1)-pre_Ang(1))/500;
    BAng=(next_Ang(2)-pre_Ang(2))/500;
    CAng=(next_Ang(3)-pre_Ang(3))/500;
    pA=AAng/AAng;
    pB=BAng/AAng;
    pC=CAng/AAng;
    count=1;
    if(AAng==0) 
       pA=AAng/BAng;
       pB=BAng/BAng;
       pC=CAng/BAng;
       count=2;
       if(BAng==0)
           pA=AAng/CAng;
           pB=BAng/CAng;
           pC=CAng/CAng;
           count=3;
       end
   end
    
    temp=pre_Ang;
    temp1=pre_Ang;
for i=1:500;
    %%沒有速度規劃
    if(next_Ang(1)>=temp(1))
        pre_Ang(1)=temp(1);
        temp(1)=pre_Ang(1)+0.5;
    end
    if(next_Ang(1)<=temp(1))
        pre_Ang(1)=temp(1);
        temp(1)=pre_Ang(1)-0.5;
    end
    if(next_Ang(2)>=temp(2))
        pre_Ang(2)=temp(2);
        temp(2)=pre_Ang(2)+0.5;
    end
    if(next_Ang(2)<=temp(2))
        pre_Ang(2)=temp(2);
        temp(2)=pre_Ang(2)-0.5;
    end
    if(next_Ang(3)>=temp(3))
        pre_Ang(3)=temp(3);
        temp(3)=pre_Ang(3)+0.5;
    end
    if(next_Ang(3)<=temp(3))
        pre_Ang(3)=temp(3);
        temp(3)=pre_Ang(3)-0.5;
    end
    pG=pos(pre_Ang,Data);
    G=pos(temp,Data);
   % plot3([pG(1);G(1)],[pG(2);G(2)],[pG(3);G(3)],'-r');
    %%有速度規劃
    pre_Ang=temp1;
    if(i>=2)
        if(count==1)
            nxt=[pre_Ang(1)+(i-1)*pA*AAng,pre_Ang(2)+(i-1)*pB*AAng,pre_Ang(3)+(i-1)*pC*AAng];
            pre=[pre_Ang(1)+i*pA*AAng,pre_Ang(2)+i*pB*AAng,pre_Ang(3)+i*pC*AAng];
        end
        if(count==2)
            nxt=[pre_Ang(1)+(i-1)*pA*BAng,pre_Ang(2)+(i-1)*pB*BAng,pre_Ang(3)+(i-1)*pC*BAng];
            pre=[pre_Ang(1)+i*pA*BAng,pre_Ang(2)+i*pB*BAng,pre_Ang(3)+i*pC*BAng];
        end
        if(count==3)
            nxt=[pre_Ang(1)+(i-1)*pA*CAng,pre_Ang(2)+(i-1)*pB*CAng,pre_Ang(3)+(i-1)*pC*CAng];
            pre=[pre_Ang(1)+i*pA*CAng,pre_Ang(2)+i*pB*CAng,pre_Ang(3)+i*pC*CAng];
        end
    ppoi=pos(pre,Data);
    poi=pos(nxt,Data);
 %   plot3([ppoi(1);poi(1)],[ppoi(2);poi(2)],[ppoi(3);poi(3)],'-g');
    end
end
end
function [A,B,C,a,b,c,Ang]=neg(XYZ,Perspective,Data)
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
XYZ;
if(move==1)
Draw(A,B,C,a,b,c,ThitaA,ThitaB,ThitaC,Perspective,Data);
end
Ang=[ThitaA,ThitaB,ThitaC]
end
function Draw(A,B,C,a,b,c,ThitaA,ThitaB,ThitaC,Perspective,Data,color)
Ta=[0+A(1),-Data(1,4)*cosd(ThitaA)+A(2),Data(1,4)*sind(ThitaA)+A(3)];
Tb=[-Data(1,4)*cosd(ThitaB)*cosd(30)+B(1),Data(1,4)*cosd(ThitaB)*sind(30)+B(2),Data(1,4)*sind(ThitaB)+B(3)];
Tc=[Data(1,4)*cosd(ThitaC)*cosd(30)+C(1),Data(1,4)*cosd(ThitaC)*sind(30)+C(2),Data(1,4)*sind(ThitaC)+C(3)];
hold on
grid on
plot3([A(1);Ta(1)],[A(2);Ta(2)],[A(3);Ta(3)],color);
plot3([a(1);Ta(1)],[a(2);Ta(2)],[a(3);Ta(3)],color);

plot3([B(1);Tb(1)],[B(2);Tb(2)],[B(3);Tb(3)],color);
plot3([b(1);Tb(1)],[b(2);Tb(2)],[b(3);Tb(3)],color);

plot3([C(1);Tc(1)],[C(2);Tc(2)],[C(3);Tc(3)],color);
plot3([c(1);Tc(1)],[c(2);Tc(2)],[c(3);Tc(3)],color);

plot3([a(1);b(1)],[a(2);b(2)],[a(3);b(3)],color);
plot3([c(1);b(1)],[c(2);b(2)],[c(3);b(3)],color);
plot3([a(1);c(1)],[a(2);c(2)],[a(3);c(3)],color);
axis([-200,200,-200,200,-300,50]);
%axis manual
view(Perspective(1),Perspective(2))
end