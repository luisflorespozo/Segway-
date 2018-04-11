
% CONTROL MANUAL Y AUTOM�TICO DE P�NDULO INVERTIDO
%
% Este script ejecuta la simulaci�n de un p�ndulo que puede ser controlado manualmente (accionando
% una barra deslizante o "slider") o de forma autom�tica, mediante lazos de control PID. El control
% autom�tico incorpora un doble lazo incluyendo un lazo con control del �ngulo del p�ndulo (para que se sit�e en
% posici�n vertical) y otro lazo m�s lento para hacer que la posici�n de la base se sit�e en un punto dado 
% en el estado de equilibrio. 
%
% El script puede ser utilizado tanto como un "equipo de pr�cticas", en el que pueden 
% estudiarse diversos conceptos de control, as� como servir de base para que el profesor (o el
% alumno) elaboren otros ejemplos interactivos totalmente distintos mediante el procedimiento 
% de "cortar y pegar".
%
%
% Fecha: 2006-11-02
% Autor: Ignacio D�az
% Area de Ingenier�a de Sistemas y Autom'�tica
% Universidad de Oviedo


clear;
close all;
clc;

disp('Instrucciones:');
disp('- Pulsar ''0'' para desactivar el sistema de control');
disp('- Pulsar otra tecla para restaurar el sistema de control');
disp(' ');
disp('Control manual: actuar sobre la barra deslizante para modificar la base del p�ndulo.');
pause(2);    


% PAR�METROS DEL P�NDULO
l = 1.70;          % Longitud del p�ndulo
m = 54;          % Masa del p�ndulo
J = m*l^2;      % Momento de inercia referido al eje
B = 1;        % Coeficiente de fricci�n
g = 9.8;         % Aceleraci�n de la gravedad


% ESTADO INICIAL DEL P�NDULO
x = [pi-0.1;0];     % Para que se vea el efecto del control, empezamos 
                    % con el p�ndulo casi vertical (theta = pi +/- "algo")


% DEFINICI�N DE UN "SLIDER"  PARA CONTROLAR MANUALMENTE EL P�NDULO
f = figure(1);
set(f,'pos',[200,200,700 700]);
h = uicontrol('style','slider','pos',[20 20 680 20],'min',-4,'max',4);


Tm = 0.01;              % Per�odo de muestreo


e0  = zeros(2,1);       % Condiciones iniciales del control de �ngulo
ep0  = zeros(2,1);      % Condiciones iniciales del control de posici�n

x0 = [0;0];             % Condiciones iniciales del p�ndulo
a0 = [0;0];

xmin = -2;
xmax = +2;
y = x(1);


% PAR�METROS DEL CONTROL PID DEL �NGULO
Kp = 10;
Ki = 100;
Kd = .1;
s = tf('s');
Cth = c2d(Kp + Ki/s + Kd*s/(0.01*s+1),Tm,'tustin');


% PAR�METROS DEL CONTROL PID DE LA POSICI�N DE LA BASE
Cpos = c2d(.04 + 0.0001/s + s*0.0001/(0.01*s+1),Tm,'tustin');


pos = 0;    % Valor inicial de la posici�n de la base del p�ndulo
k = 2;      % Empezamos en k=2 para tener acceso al menos a dos muestras anteriores
while 1,
    k = k + 1;
    
    % BUFFER CON LA POSICI�N (X(3) TIENE LA POSICI�N ACTUAL)
    % X(k) = get(h,'Value');

    e(k) = pi-y;
    
    % RESTRINGIMOS EL VALOR DEL ERROR AL INTERVALO [-pi,pi]
    e(k) = mod(e(k)+pi,2*pi)-pi;
    
 
    % CONTROL DE LA POSICI�N DE LA BASE DEL P�NDULO
    ep = 0-pos;
    [dpos,ep0] = filter(Cpos.num{1},Cpos.den{1},ep,ep0);
    e(k) = e(k) - dpos;
    
    
    % SELECCI�N DE CONTROL MANUAL / CONTROL AUTOM�TICO
    tecla = get(f,'currentchar');
    switch tecla
        case '0'
            pos = get(h,'value');
        otherwise
            [pos,e0] = filter(Cth.num{1},Cth.den{1},e(k),e0);
    end
    

    
    % Suavizado de la aceleraci�n (muy conveniente, porque el movimiento del 
    % objeto "slider" con un rat�n se produce a saltos, dando lugar a segundas 
    % derivadas muy elevadas)
    [X(k),x0] = filter(.01,poly([.9 .9]),pos,x0);
    [a,a0] = filter((1/Tm^2)*[1 -2 1],[1 0 0],X(k),a0);
    A(k) = a;
    
    
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ECUACIONES EN ESPACIO DE ESTADOS (NO LINEALES) DEL P�NDULO
    u = -a;         % Asignamos la entrada

    % Ecuaci�n de estados
    x(1) = x(1) + Tm*x(2);
    x(2) = x(2) + Tm*(1/J*(-B*x(2)-m*g*l*sin(x(1))+m*u*l*cos(x(1))));
    % Ecuaci�n de salida
    y    = x(1);
    
    
    th = y;         % Asignamos la salida
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    
    
    % REPRESENTACI�N GR�FICA DE LA SIMULACI�N
    figure(1);
    plot(X(k),0,'.');
    hold on;
    p1 = X(k);
    p2 = X(k)+l*exp(j*(th-pi/2));
    line(real([p1,p2]),imag([p1,p2]));
    plot(real(p2),imag(p2),'.','markersize',40);

    hold off;
    % Sugerencia: pueden dibujarse tambi�n otras flechas indicando en tiempo real las fuerzas reales 
    % o de inercia que act�an en cada elemento del sistema

    % Centrado autom�tico de la perspectiva sobre el objeto de control
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