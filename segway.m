
% CONTROL MANUAL Y AUTOMÁTICO DE PÉNDULO INVERTIDO
%
% Este script ejecuta la simulación de un péndulo que puede ser controlado manualmente (accionando
% una barra deslizante o "slider") o de forma automática, mediante lazos de control PID. El control
% automático incorpora un doble lazo incluyendo un lazo con control del ángulo del péndulo (para que se sitúe en
% posición vertical) y otro lazo más lento para hacer que la posición de la base se sitúe en un punto dado 
% en el estado de equilibrio. 
%
% El script puede ser utilizado tanto como un "equipo de prácticas", en el que pueden 
% estudiarse diversos conceptos de control, así como servir de base para que el profesor (o el
% alumno) elaboren otros ejemplos interactivos totalmente distintos mediante el procedimiento 
% de "cortar y pegar".
%
%
% Fecha: 2006-11-02
% Autor: Ignacio Díaz
% Area de Ingeniería de Sistemas y Autom'ática
% Universidad de Oviedo


clear;
close all;
clc;

disp('Instrucciones:');
disp('- Pulsar ''0'' para desactivar el sistema de control');
disp('- Pulsar otra tecla para restaurar el sistema de control');
disp(' ');
disp('Control manual: actuar sobre la barra deslizante para modificar la base del péndulo.');
pause(2);    


% PARÁMETROS DEL PÉNDULO
l = 1.70;          % Longitud del péndulo
m = 54;          % Masa del péndulo
J = m*l^2;      % Momento de inercia referido al eje
B = 1;        % Coeficiente de fricción
g = 9.8;         % Aceleración de la gravedad


% ESTADO INICIAL DEL PÉNDULO
x = [pi-0.1;0];     % Para que se vea el efecto del control, empezamos 
                    % con el péndulo casi vertical (theta = pi +/- "algo")


% DEFINICIÓN DE UN "SLIDER"  PARA CONTROLAR MANUALMENTE EL PÉNDULO
f = figure(1);
set(f,'pos',[200,200,700 700]);
h = uicontrol('style','slider','pos',[20 20 680 20],'min',-4,'max',4);


Tm = 0.01;              % Período de muestreo


e0  = zeros(2,1);       % Condiciones iniciales del control de ángulo
ep0  = zeros(2,1);      % Condiciones iniciales del control de posición

x0 = [0;0];             % Condiciones iniciales del péndulo
a0 = [0;0];

xmin = -2;
xmax = +2;
y = x(1);


% PARÁMETROS DEL CONTROL PID DEL ÁNGULO
Kp = 10;
Ki = 100;
Kd = .1;
s = tf('s');
Cth = c2d(Kp + Ki/s + Kd*s/(0.01*s+1),Tm,'tustin');


% PARÁMETROS DEL CONTROL PID DE LA POSICIÓN DE LA BASE
Cpos = c2d(.04 + 0.0001/s + s*0.0001/(0.01*s+1),Tm,'tustin');


pos = 0;    % Valor inicial de la posición de la base del péndulo
k = 2;      % Empezamos en k=2 para tener acceso al menos a dos muestras anteriores
while 1,
    k = k + 1;
    
    % BUFFER CON LA POSICIÓN (X(3) TIENE LA POSICIÓN ACTUAL)
    % X(k) = get(h,'Value');

    e(k) = pi-y;
    
    % RESTRINGIMOS EL VALOR DEL ERROR AL INTERVALO [-pi,pi]
    e(k) = mod(e(k)+pi,2*pi)-pi;
    
 
    % CONTROL DE LA POSICIÓN DE LA BASE DEL PÉNDULO
    ep = 0-pos;
    [dpos,ep0] = filter(Cpos.num{1},Cpos.den{1},ep,ep0);
    e(k) = e(k) - dpos;
    
    
    % SELECCIÓN DE CONTROL MANUAL / CONTROL AUTOMÁTICO
    tecla = get(f,'currentchar');
    switch tecla
        case '0'
            pos = get(h,'value');
        otherwise
            [pos,e0] = filter(Cth.num{1},Cth.den{1},e(k),e0);
    end
    

    
    % Suavizado de la aceleración (muy conveniente, porque el movimiento del 
    % objeto "slider" con un ratón se produce a saltos, dando lugar a segundas 
    % derivadas muy elevadas)
    [X(k),x0] = filter(.01,poly([.9 .9]),pos,x0);
    [a,a0] = filter((1/Tm^2)*[1 -2 1],[1 0 0],X(k),a0);
    A(k) = a;
    
    
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ECUACIONES EN ESPACIO DE ESTADOS (NO LINEALES) DEL PÉNDULO
    u = -a;         % Asignamos la entrada

    % Ecuación de estados
    x(1) = x(1) + Tm*x(2);
    x(2) = x(2) + Tm*(1/J*(-B*x(2)-m*g*l*sin(x(1))+m*u*l*cos(x(1))));
    % Ecuación de salida
    y    = x(1);
    
    
    th = y;         % Asignamos la salida
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    
    
    % REPRESENTACIÓN GRÁFICA DE LA SIMULACIÓN
    figure(1);
    plot(X(k),0,'.');
    hold on;
    p1 = X(k);
    p2 = X(k)+l*exp(j*(th-pi/2));
    line(real([p1,p2]),imag([p1,p2]));
    plot(real(p2),imag(p2),'.','markersize',40);

    hold off;
    % Sugerencia: pueden dibujarse también otras flechas indicando en tiempo real las fuerzas reales 
    % o de inercia que actúan en cada elemento del sistema

    % Centrado automático de la perspectiva sobre el objeto de control
    if X(k)>xmax-1
        xmin = xmin + 0.1;
        xmax = xmax + 0.1;
    elseif X(k)<xmin+1
        xmin = xmin - 0.1;
        xmax = xmax - 0.1;
    end
    grid on;
    axis([xmin-3 xmax+3 -5 5]);

    % Refresco de la imagen
    drawnow;

end