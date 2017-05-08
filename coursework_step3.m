% Clear all the variables
clear all
load('assessed_tutorial_data.mat');
% Initial configuration
theta=zeros(1,6);
% DH parameters
a2=0.4318;
a3=0.0202;
d3=0.1244;
d4=0.7;  %changed this value and it followed correct path
% DH Table -> [theta d a alpha]
Link1=Link([theta(1),0,0,0],'modified');
Link2=Link([theta(2),0,0,-pi/2],'modified');
Link3=Link([theta(3),d3,a2,0],'modified');
Link4=Link([theta(4),d4,a3,-pi/2],'modified');
Link5=Link([theta(5),0,0,pi/2],'modified');
Link6=Link([theta(6),0,0,-pi/2],'modified');
% Define the robot structure
Links=[Link1,Link2,Link3,Link4,Link5,Link6];
% Robot object
PUMA_cw=SerialLink(Links);
% TransfoPUMA_cwation from tool frame to global frame
%R_tool=rotz(pi/2);
% Assignment of tool frame 
%PUMA_cw.tool=r2t(R_tool);
% Desired pose of the robot end-effector
% a = 0.9848;
% b = sqrt(1-a^2); %to make the marix orthonoPUMA_cwal
% T=[ a b 0 0.2614;
%  -b a 0 0.1302;
%  0 0 1.0000 0;
%  0 0 0 1.0000];

[m n] = size(pos_static_trajectory); %obtain matrix size
R = zeros (3,3,n); %3d matrix containing n 2d arrays for storing rotation matrices
TR = zeros (4,4,n); %3d matrix containing n 2d arrays for storing homogenous egns
x = zeros (4,4,n); %3d matrix containing n 2d arrays for storing homogenous egns
q = zeros (6,28);
num = 1; %num is the index number of q 

for L=1:1:n-1 %loop to calculate distance and time variables
    %next line calculates the distance covered between each point and
    %stores in an array
    distance(L) = sqrt((pos_static_trajectory(1,L)- pos_static_trajectory(1,L+1))^2 + (pos_static_trajectory(2,L)- pos_static_trajectory(2,L+1))^2);
end
%next line calculates total distance covered
total_distance = sum(distance);
%wanted velocity = .1m/s therefore total time spent = distance/speed
time = 0.1/total_distance;
%we know that q will have 29 different matrices so each iteration of q
%should take 1/29th of the total time meaning 26 inbetween plus 0 and time
%inclusive
Dt = time/27;
Time_Vector = (0:Dt:time);

for i=1:1:n %for loop to set trajectory points
    %for increasing values of 1, fill the columns with the trajectory
    %points
    point(i,:) = [pos_static_trajectory(1,i); pos_static_trajectory(2,i); 0]; 
    %create arrays of orthonormal rotation matrix, R, equivalent to a rotation of THETA about the vector V.
    R(:,:,i) = angvec2r(alpha(i),vector);
    %the next line converts rotation matrix R and translation matrix point to homogeneous transform
    TR(:,:,i) = rt2tr(R(:,:,i), point(i,:).'); %the " .' " is added to find the transpose of point
    
    %next 4 lines is an attempt to make TR orthonormal for first iteration
    %of q;
    a = 0.8290;  %TR(1,1,i); 
    b = sqrt(1-a^2);
    TR(1,2,i) = b;
    TR(2,1,i) = -b;
    TR(1,1,i) = a;
    TR(2,2,i) = a;
    x(:,:,i) = transl(point(i,:));%this puts the trajectory points as homogenous transforms in a 3d array 
end

for j=0:Dt:time %total time vector
    if num == 1
        %the next line is used to estimate the inverse kinematics and 
        %joint angles for the first position of robot
        q(:,num) = PUMA_cw.ikine(TR(:,:,num));
    else
        %use iterative inverse kinematics to find the angles using the ffg
        %steps:
        %1) Find the inverse jacobian of the last value of q
        J0=PUMA_cw.jacob0(q(:,num-1));
        Inverse = inv(J0(1:3,1:3));
        %2)Use iterative inverse kinematics eqn to calculate the next q
        %NOTE: From DH table and homogenous transform matrix 0T6, we can
        %see that q4, q5 and q6 do not affect translation so do not need a
        %jacobian calculation as they are constant
        q(1:3,num) =  q(1:3,num-1) + (Inverse * ((point(num,:).') - (point(num-1,:).')));
        q(4,num) = q(4,num-1);
        q(5,num) = q(5,num-1);
        q(6,num) = q(6,num-1);
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%         CODE USED TO FIND Xa, Ya, and Za
%         %find matrix eqn using symbolic toolbox
%         syms t1 t2 t3 t4 t5 t6 a2 a3 d2 d3 d4
%         A = [cos(t1) -sin(t1) 0 0; sin(t1) cos(t1) 0 0;0 0 1 0; 0 0 0 1];
%         B = [cos(t2) -sin(t2) 0 0; 0 0 1 d2; -sin(t2) -cos(t2) 0 0; 0 0 0 1];
%         C = [cos(t3) -sin(t3) 0 a2; sin(t3) cos(t3) 0 0;0 0 1 d3; 0 0 0 1];
%         D = [cos(t4) -sin(t4) 0 a3; 0 0 -1 -d4; sin(t4) cos(t4) 0 0; 0 0 0 1];
%         E = [cos(t5) -sin(t5) 0 0; 0 0 1 0; -sin(t5) -cos(t5) 0 0; 0 0 0 1];
%         F = [cos(t6) -sin(t6) 0 0; 0 0 -1 0; sin(t6) cos(t6) 0 0; 0 0 0 1];
%         Forward_kinematics = A * B * C * D * E * F;
%         Forward_kinematics(:,4)
%
%         %where Xa = d4*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)) - a3*(cos(t1)*sin(t2)*sin(t3)
%         % - cos(t1)*cos(t2)*cos(t3)) - d2*sin(t1) - d3*sin(t1) + a2*cos(t1)*cos(t2)
%
%         %Ya = d4*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - a3*(sin(t1)*sin(t2)*sin(t3) 
%         % - cos(t2)*cos(t3)*sin(t1)) + d2*cos(t1) + d3*cos(t1) + a2*cos(t2)*sin(t1)
%
%         %Za = d4*(cos(t2)*cos(t3) - sin(t2)*sin(t3)) - a3*(cos(t2)*sin(t3) + cos(t3)*sin(t2)) - a2*sin(t2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

           %now we find joint velocity using the inverse jacobian and
           %linear velocity
           %linear velocity = 0.1m/s diagonally meaning x and y variables will be
           %calculated using pythagoras to give x_vel = y_vel = sqrt(2)/20
           x_vel = sqrt(2)/20;
           y_vel = sqrt(2)/20;
           vel = [x_vel;y_vel;0]; %assume z velocity is 0 as no distance in z axis
           %compute joint velocities
           q_dot(1:3,num-1) = Inverse * vel;
           q_dot(4:6,num-1) = 0;
    end
    num = num + 1;
end

% Plot of joint angles during motion
subplot(2,5,3)
plot(Time_Vector,q(1,:),'r--')
xlabel('Time [s]')
ylabel('Theta_1 [rad]')
subplot(2,5,4)
plot(Time_Vector,q(2,:),'g--')
xlabel('Time [s]')
ylabel('Theta_2 [rad]')
subplot(2,5,5)
plot(Time_Vector,q(3,:),'b--')
xlabel('Time [s]')
ylabel('Theta_3 [rad]')
% Plot of joint velocities during motion
subplot(2,5,8)
plot(Time_Vector(1:end-1),q_dot(1,:),'r--')
xlabel('Time [s]')
ylabel('dTheta_1/dt [rad/s]')
subplot(2,5,9)
plot(Time_Vector(1:end-1),q_dot(2,:),'g--')
xlabel('Time [s]')
ylabel('dTheta_2/dt [rad/s]')
subplot(2,5,10)
plot(Time_Vector(1:end-1),q_dot(3,:),'b--')
xlabel('Time [s]')
ylabel('dTheta_3/dt [rad/s]')
% Plot of robot during motion
subplot(2,5,[1 2 6 7])
plot(point(:,1),point(:,2),'ro')
%view(3)
axis equal
axis([-1 2 -1 2])
hold on
%plot robot
PUMA_cw.plot(q');