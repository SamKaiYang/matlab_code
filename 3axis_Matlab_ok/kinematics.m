function kinematics(Od,Link)
%% ?i Step ???j ?w?q???u???????PDH?s?????????P????????????
        % ???u????????
        %???u???U?b????:d1=10 d2=10 d3=10         
%% ?i Step 2 ?j ?p???f?B????
         
         JointAngle=InverseKinematics(Od,Link)
          
%% ?i Step 3 ?j ?p?????B????        
        % DH ??????        -->  % ????DHparameter( )          
        % ????????????????  -->   % ????GenerateTransformationMatrices( )
        %JointAngle = [0 90 0 0 90 0]
        pos=ForwardKinematics(JointAngle)
%% ?i Step 4 ?j ???s???u
DrawRobotManipulator(pos)
end


% =========================================================================
%                              ?iFunctions?j 
% =========================================================================
%%  DH ??????
function DH= DHparameter(JointAngle)
% ???b?? DH ???? [ a      ?\        d       ?c ]
% DH=[0 90 10 JointAngle(1)+90;
%     10 0 0 JointAngle(2);
%     0 -90 0 JointAngle(3)-90;
%     0 90 10 JointAngle(4);
%     0 -90 0 JointAngle(5);
%     0 0 10 JointAngle(6)]
DH=[0 90 10 JointAngle(1)+90;
    10 0 0 JointAngle(2);
    10 90 0 JointAngle(3)
    ]
end
%%  ?????????x?}
function  Tran=GenerateTransformationMatrices(DH,start,stop)
                   Tranoriginal=eye(4);
	for i=start:stop
	tran=[cosd(DH(i,4)) -sind(DH(i,4))*cosd(DH(i,2)) sind(DH(i,4))*sind(DH(i,2)) DH(i,1)*cosd(DH(i,4));
    	      sind(DH(i,4)) cosd(DH(i,4))*cosd(DH(i,2)) -cosd(DH(i,4))*sind(DH(i,2)) DH(i,1)*sind(DH(i,4));
    		                      0               sind(DH(i,2))             cosd(DH(i,2))      DH(i,3);
   					   0               0                         0                  1];
	Tranoriginal=Tranoriginal*tran;
	Tran=Tranoriginal;
	end
    
end
%% ???B????
function pos= ForwardKinematics(JointAngle)
    DH=DHparameter(JointAngle);
	Tran1=GenerateTransformationMatrices(DH,1,1);
	Tran2=GenerateTransformationMatrices(DH,1,2);
	Tran3=GenerateTransformationMatrices(DH,1,3);
  
    pos=[0 0 0;
        Tran1(1,4) Tran1(2,4) Tran1(3,4);
        Tran2(1,4) Tran2(2,4) Tran2(3,4);
        Tran3(1,4) Tran3(2,4) Tran3(3,4)
     ]
   
    
    
end
%% ?f?B????1~3????
function  JointAngle=InverseKinematics(Od,Link)
Oc = Od
Link = [10 10 10]
JointAngle(1) = -atan2(Oc(1), Oc(2)) 
D = norm(Oc-[0, 0, Link(1)])
%fprintf(D,%d)
JointAngle(2) = 0
JointAngle(3) =acos(((Link(2))^2 + Link(3)^2  - D^2 )/(2*Link(2)*Link(3))) - pi
JointAngle(2) =atan2((Oc(3)-Link(1)),sqrt(Oc(1)^2+Oc(2)^2)) - atan2((Link(3)*sin(JointAngle(3))),(Link(2)+Link(3)*cos(JointAngle(3))));
JointAngle=JointAngle*180/pi
%JointAngle = JointAngle*pi/180
end        
%%  ?e?? ?e???????u ???J: ???????A?U???`?????m?A?U???`???y??
function  DrawRobotManipulator(pos)
%axis([-50,50,-40,80,-20,80])
axis([-100,100,-100,100,0,100])
temp=pos
hold on
xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis');
plot3(temp(:,1),temp(:,2),temp(:,3),'bo-','linewidth',3,'markeredgecolor','r','markerfacecolor','r','markersize',10);
grid on
end