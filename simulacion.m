%%Declara robot propio
q1=0;
q2=0;
q3=0;
q4=0;
q5=0 +pi/2;
theta1=0;
theta2=0;
theta3=0;
theta4=0;
theta5=0;

% L1 = Link([theta d a alpha); % Parámetros DH estándar
% L1 = Link([theta d a alpha], 'standard');
% L2 = Link([0 0 0.4 0]);


%Robot en vertical
figure
v1 = Link([0 134 18 pi/2],Revolute);
v2 = Link([0 0 120 0]);
v3 = Link([0 0 0 -pi/2]);
v4 = Link([0 200 0 pi/2]);
v5 = Link([0 0 110 0]);
robot1 = SerialLink([v1 v2 v3 v4 v5], 'name', 'MiRobot_vertical');
robot1.plot([q1 q2 q3 q4 q5]);
%   % Visualiza con ángulos de las articulaciones q1 y q2
% %ROBOT EN HORIZONTAL
% figure
% H1 = Link([theta1 134 18 pi/2],Revolute);
% H2 = Link([theta2 0 120 0]);
% H3 = Link([pi/2+theta3 0 0 pi/2]);
% H4 = Link([theta4 0 125 pi/2]);
% H5 = Link([-pi/2+theta5 0 110 pi/2]);
% robot = SerialLink([H1 H2 H3 H4 H5], 'name', 'MiRobot_horizontal');
% robot.plot([q1 q2 q3 q4 q5]); % Visualiza con ángulos de las articulaciones q1 y q2


% % % % Matrices de transformacion
T01=[[cos(theta1), 0,   sin(theta1),    18.0*cos(theta1)];
    [sin(theta1),  0,   -1*cos(theta1), 18.0*sin(theta1)];
    [0,            1.0,  0,             134];
    [0,            0,    0,             1]];

T12=[[cos(theta2), -1.0*cos(theta2), 0,    120.0*cos(theta2)];
    [sin(theta2),  cos(theta2),     0,    120.0*sin(theta2)];
    [0,            0,               1.0,  0];
    [0,            0,               0,    1]];

T23=[[cos(theta3), 0,    -1.0*sin(theta3), 0];
    [sin(theta3),  0,    cos(theta3),      0];
    [0,            1.0,  0,                0];
    [0,            0,    0,                1]];

    
T34=[[cos(theta4), 0,    sin(theta4),      0];
    [sin(theta4),  0,    -1.0*cos(theta4), 0];
    [0,            1.0,  0,                200.0];
    [0,            0,    0,                1]];

T45=[[cos(theta5+1.571), -1*sin(theta5+1.571), 0,   110.0*cos(theta5+1.571)];
    [sin(theta5+1.571),  cos(theta5+1.571),    0,   110.0*cos(theta5+1.571)];
    [0,            1.0,                        0,   0];
    [0,            0,                          0,   1]];

% % % % Obtencion de matrices T0N%%%%%
T02=T01*T12;
T03=T01*T12*T23;
T04=T01*T12*T23*T34;
T05=T01*T12*T23*T34*T45;

% % % Obtencion de los zn y Tn
Z0=[0;0;1];
T0=[0;0;0];

Z1=T01(1:3,3);
T1=T01(1:3,4);

Z2=T02(1:3,3);
T2=T02(1:3,4);

Z3=T03(1:3,3);
T3=T03(1:3,4);

Z4=T04(1:3,3);
T4=T04(1:3,4);

Z5=T05(1:3,3);
T5=T05(1:3,4);


% % % % Calculo del jacobiano

Jv= [cross(Z0,(T5-T0)),cross(Z1,(T5-T1)),cross(Z2,(T5-T2)),cross(Z3,(T5-T3)),cross(Z4,(T5-T4))];

Jacob=@(theta_1,theta_2,theta_3,theta_4,theta_5)[cross(Z0,(T5-T0)),cross(Z1,(T5-T1)),cross(Z2,(T5-T2)),cross(Z3,(T5-T3)),cross(Z4,(T5-T4))];
Jw=[Z0,Z1,Z2,Z3,Z4];

Jaco= [Jv;Jw];
q=[pi/4 pi/4 pi/4 pi/4 pi/4]';
t=0.01;
N=1000;
Q=zeros(2,N);
for i=1:N
    xp=[0;1;0];
    J=Jacob(q(1),q(2),q(3),q(4),q(5));
    qp=pinv(J)*xp;
    q=q+qp*t;
    Q(:,i)=q;
end
bot.plot(Q')

