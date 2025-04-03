clear arduinoObj
arduinoObj = serialport("COM3", 9600);  % Cambia "COM3" según tu puerto
v1 = Link([0 13.4 1.8 pi/2],Revolute);
v2 = Link([0 0 12.0 0]);
v3 = Link([0 0 0 -pi/2]);
v4 = Link([0 13.0 0 pi/2]);
v5 = Link([0 0 9.0 0]);
robot1 = SerialLink([v1 v2 v3 v4 v5], 'name', 'MiRobot_vertical');

 q=[0 pi/2 -3*pi/4 0 pi/4]';
 q=[0 0 0 0 0]';
t=0.01;
N=50; 
td=[30 0 10]';
theta=pi/2;
Rd=[1 0 0 ; 0 cos(theta) -sin(theta) ; 0    sin(theta)    cos(theta) ];
    
K=[1,0,0;
    0,1,0;
    0,0,1];    
Kg=diag([3,3,3,3,3,3]);
Q=zeros(5,N);
qplot=zeros(5,N);
qp_plot=zeros(5,N);
x_plot=zeros(3,N);
xd_plot=zeros(3,N);
m_plot=zeros(1,N);

for i=1:N
theta_1=q(1);
theta_2=q(2);
theta_3=q(3);
% theta_3=-theta_3;
theta_4=q(4);
theta_5=q(5);
    
T01=[[cos(theta_1), 0,   sin(theta_1),    1.8*cos(theta_1)];
    [sin(theta_1),  0,   -1.0*cos(theta_1), 1.8*sin(theta_1)];
    [0,            1.0,  0,             13.4];
    [0,            0,    0,             1.0]];

T12=[[cos(theta_2), -1.0*sin(theta_2), 0,    12.0*cos(theta_2)];
    [sin(theta_2),  cos(theta_2),     0,    12.0*sin(theta_2)];
    [0,            0,               1.0,  0];
    [0,            0,               0,    1.0]];

T23=[[cos(theta_3), 0,    -1.0*sin(theta_3), 0];
    [sin(theta_3),  0,    cos(theta_3),      0];
    [0,            -1.0,  0,                0];
    [0,            0,    0,                1.0]];

    
T34=[[cos(theta_4), 0,    sin(theta_4),      0];
    [sin(theta_4),  0,    -1.0*cos(theta_4), 0];
    [0,            1.0,  0,                13.00];
    [0,            0,    0,                1.0]];

T45=[[cos(theta_5), -1.0*sin(theta_5), 0,   9.0*cos(theta_5)];
    [sin(theta_5),  cos(theta_5),    0,   9.0*sin(theta_5)];
    [0,            0,                        1.0,   0];
    [0,            0,                          0,   1.0]];
T02=T01*T12;
T03=T02*T23;
T04=T03*T34;
T05=T04*T45;
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



Jacob=[cross(Z0,(T5-T0)),cross(Z1,(T5-T1)),cross(Z2,(T5-T2)),cross(Z3,(T5-T3)),cross(Z4,(T5-T4))];
Jw=[Z0,Z1,Z2,Z3,Z4];
J=[Jacob;Jw];
    % Tau=robot1.fkine(q)
    % Tau=Tau.T;
    ti=T05(1:3,4);
    Ri=T05(1:3,1:3);
    % xp=[0;10;0];

    % J=Jacob(q(1),q(2),q(3),q(4),q(5));
    v=td-ti;
    w=0.5*(cross(Ri(1:3,1),Rd(1:3,1))+ cross(Ri(1:3,2),Rd(1:3,2))+ cross(Ri(1:3,3),Rd(1:3,3)));
    % Ji=inv(J'*J)*J';

    % qp=Ji*K*v;
     qp=pinv(J)*Kg*[v;w];
    % qp = (J' * J + lambda * eye(size(J,2))) \ (J' * Kg * [v; w]);
    q=q+qp*t
    q = mod(q + pi, 2*pi) - pi;
 % Restricción al rango [0, pi]
    % Supongamos que tienes una función que calcula la distancia mínima entre enlaces:
% d_min = calcular_distancia_minima(q);  % Debes definir esta función según la geometría del robot
% umbral = 5; % umbral en unidades apropiadas
% 
% if d_min < umbral
%     % Calcula una corrección en el espacio nulo que aumente la distancia
%     grad_C = calcular_gradiente_distancia(q); % Función que debes definir
%     % Proyecta la corrección en el espacio nulo de la tarea principal:
%     N_space = eye(5) - pinv(J)*J;  % espacio nulo del Jacobiano
%     qp_correccion = -0.1 * N_space * grad_C;  % factor de corrección (ajustable)
% 
%     qp = qp + qp_correccion;  % ajusta la velocidad articular
% end

% % Actualización de q
% q = q + qp * t;
%  q = max(min(q, pi), -pi); % Limita q al rango [0, pi]




    
    
    


% Enviar por serial
  

    Q(:,i)=q;

    qplot(:,i)=q;
    qp_plot(:,i)=qp;
    x_plot(:,i)=ti;
    xd_plot(:,i)=td;

    m=sqrt(det(J'*J));
    m_plot(i)=m;

end

robot1.plot(Q')

  q_A= rad2deg(q)
    q_A(3)=-q_A(3)
    

    % Vector de ángulos en grados (ejemplo)
angulos = q_A;

% Asegurarse de que los ángulos estén en el rango 0-180
for i = 1:length(angulos)
    % Si el ángulo es negativo, transformarlo a 180 - ángulo
    if angulos(i) < 0
        angulos(i) = 180 - abs(angulos(i));  % Si el ángulo es negativo, lo transformamos
    end
    % Asegurarse de que los ángulos estén en el rango 0-180
    angulos(i) = mod(angulos(i), 180);  % Para que los valores negativos se ajusten a 0-180
end

% Enviar datos al Arduino
for i = 1:length(angulos)
    % Convertir ángulo a cadena y enviarlo
    dataStr = sprintf("A%d:%.2f\n", i, angulos(i));  
    writeline(arduinoObj, dataStr);
    pause(.5);  % Pequeña pausa para evitar saturación del puerto
end

disp("Datos enviados correctamente a Arduino.");

clear arduinoObj
figure
hold on
grid on
plot(xd_plot(1,:),'r-')
plot(x_plot(1,:),'b-')
title("x vs xd")

figure
hold on
grid on
plot(xd_plot(2,:),'r-')
plot(x_plot(2,:),'b-')
title("y vs yd")

figure
hold on
grid on
plot(xd_plot(3,:),'r-')
plot(x_plot(3,:),'b-')
title("z vs zd")

figure
hold on
grid on
plot(m_plot,'LineWidth',2)
title("Manipulabilidad")