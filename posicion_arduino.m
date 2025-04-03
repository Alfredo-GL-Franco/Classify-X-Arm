 clear arduinoObj
 arduinoObj = serialport("COM3", 9600);  % Cambia "COM3" según tu puerto
v1 = Link([0 13.4 1.8 pi/2]);
v2 = Link([0 0 12.0 pi]);
v3 = Link([0 0 0 pi/2]);
v4= Link([0 13 0 pi/2]);
 v5 = Link([0 0 9.0 0]);
 % v4 = Link([0 13.0 0 pi/2]);
robot1 = SerialLink([v1 v2 v3 v4 v5], 'name', 'MiRobot_vertical');

 q=[0 0 0 0 pi]';
 % q=[0 0 0 pi/2 0]';
q_A= rad2deg(q)
 q_A(3)=-q_A(3)

% Configurar comunicación serial con Arduino (ajusta el puerto COM)


% Vector de ángulos en grados (ejemplo)
angulos = q_A;

% % Asegurarse de que los ángulos estén en el rango 0-180
for i = 1:length(angulos)
    % Si el ángulo es negativo, transformarlo a 180 - ángulo
    if angulos(i) < 0
        angulos(i) = 180 - abs(angulos(i));  % Si el ángulo es negativo, lo transformamos
    end
    % Asegurarse de que los ángulos estén en el rango 0-180
    angulos(i) = mod(angulos(i), 180);  % Para que los valores negativos se ajusten a 0-180
end
% angulos = angulos(:).';  % Convertir a vector fila si no lo es
% 
% % Convertir todos los ángulos en una sola cadena
% dataStr = sprintf("%.2f,", angulos);
% dataStr = dataStr(1:end-1);  % Eliminar la última coma
% dataStr = strcat("A:", dataStr, "\n");  % Agregar identificador y salto de línea

% Enviar el mensaje completo
writeline(arduinoObj, dataStr);



disp("Datos enviados correctamente a Arduino.");



    robot1.plot(q')