function main()
clc;
clear all;
%%               逆運動學
Od= [0 20 20 0 0 0]
Link=[10 10 10 10]

kinematics(Od,Link);
%%               正運動學
% kinematics( JointAngle );
end