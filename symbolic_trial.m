%find matrix eqn using symbolic toolbox
syms t1 t2 t3 t4 t5 t6 a2 a3 d2 d3 d4
A = [cos(t1) -sin(t1) 0 0; sin(t1) cos(t1) 0 0;0 0 1 0; 0 0 0 1];
B = [cos(t2) -sin(t2) 0 0; 0 0 1 d2; -sin(t2) -cos(t2) 0 0; 0 0 0 1];
C = [cos(t3) -sin(t3) 0 a2; sin(t3) cos(t3) 0 0;0 0 1 d3; 0 0 0 1];
D = [cos(t4) -sin(t4) 0 a3; 0 0 -1 -d4; sin(t4) cos(t4) 0 0; 0 0 0 1];
E = [cos(t5) -sin(t5) 0 0; 0 0 1 0; -sin(t5) -cos(t5) 0 0; 0 0 0 1];
F = [cos(t6) -sin(t6) 0 0; 0 0 -1 0; sin(t6) cos(t6) 0 0; 0 0 0 1];
Forward_kinematics = A * B * C * D * E * F;
Forward_kinematics(:,4)
