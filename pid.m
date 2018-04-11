% CONTROL MANUAL Y AUTOMATICO DE P  ENDULO INVERTIDO 
% Este script ejecuta la simulacion de un pendulo mediante lazos de control PID. El control
% automatico incorpora un doble lazo incluyendo un lazo con control del angulo del pendulo
% (para que se situe en posicion vertical) y otro lazo mas lento para hacer que la posicion
% de la base se situe en un punto dado en el estado de equilibrio.
%
% Fecha: 2006-11-02
% Autor: Ignacio Daz
% Area de Ingeniera de Sistemas y Automatica
% Universidad de Oviedo
%
% Modicado por: Pascual Perez
% Instituto de Informatica Industrial.
% Universidad de Politecnica de Valencia
clear;
close all;
clc;
% M masa del carro 0.5 kg
% m masa del pendulo 0.5 kg
% b friccion del carro 0.1 N/m/sec
% l longitud al centro de masa del pendulo 0.3 m
% I inercia del pendulo 0.006 kg*m2^
% u fuerza escalon aplicada al carro
% x coordenadas de posicion del carro
% phi angulo del pendulo respecto de la vertical
M0=40;
M1=54;
ls=0.5;
thetas=0.008;
Fr=6.2;
C=0.009;
g=9.8;

% El carro que va con ruedas no puede acelerar mas de limitu
limitu=1e6; % no hay limite
M=M1+M0
theta=thetas+M1*ls*ls
alpha=M1*ls
beta=theta*M-alpha*alpha;
Tm=1/100
%linealizacion por taylor
a31=0;
a32=alpha*alpha*g;
a33=-theta*Fr/beta;
a34=-alpha*C/beta;
b3=theta/beta;
a41=0;
a42=alpha*M*g/beta;
a43=-alpha*Fr/beta;
a44=-M*C/beta;
b4=alpha/beta;
Am = [0 0 1 0 ; 0 0 0 1 ; a31 a32 a33 a34 ; a41 a42 a43 a44]
Bm = [ 0 ; 0 ; b3 ; b4]
% Cm nos da los valores del estado que habra que multiplicarlos por la
% matriz de ganancia para aplicar la nueva u
% se pueden medir las cuatro variables de estado
Cm = [1 0 0 0 ; 0 1 0 0 ; 0 0 1 0 ; 0 0 0 1];
Dm = [0; 0; 0; 0]; %espacio de estados discreto
[Fm,Gm,Hm,Jm]=c2dm (Am,Bm,Cm,Dm,Tm,'zoh');
% estado inicial el pendulo un poco inclinado
x=[0 ; pi/16 ; 0 ;0 ]
u=0;
% especicar la dinamica que se quiere y calcular la matriz con el
% comando lqr para el sistema discreto
% es controlable y observable el sistema
co = ctrb (Fm,Gm);
ob = obsv (Fm,Hm);
Controllability = rank (co)
Observability = rank (ob)
ppos=5000;%factor de peso para la posicion del carro
pphi=100; %factor de peso para el angulo del pendulo
Qm=[ppos 0 0 0; 0 0 0 0; 0 0 pphi 0; 0 0 0 0];
Rm = 1;
Km = dlqr(Fm,Gm,Qm,Rm)
% La matriz Cnm establece que factor de entrada se debe reeescalar
Cnm=[1 0 0 0];
Nbar=rscale(Fm,Gm,Cnm,0,Km)
f = figure(1);
set(f,'pos',[200,600,1000 250]);
u=10;
k = 2;
while ((k<20*1/Tm) && (isempty(get(f,'currentchar')==['0'])))
if ((k>0)&&(Tm*k<1)) Ref=0;
else Ref=0;
end
k = k + 1;
% REPRESENTACION GR  AFICA DE LA SIMULACI ON
% si lo dejamos trabajar no funciona puesto que la linealizacion es
% para angulos peque~nos de la inclinacion del pendulo
% como funciona la linealizacion.
% Lo mejor seria obtener linealizar el sistema en varios puntos de
% trabajo y aplicar ganancias en distintos puntos.
% para veril control simular el sistema con las ecuaciones no
% lineales, calcular las matrices de ganancia para diferentes puntos de
% trabajo y aplicarlas al calculo en bucle cerrado.
% Asignamos la entrada
% Ecuacion de estados no lineal
x(1) = x(1) + Tm*x(3);
x(2) = x(2) + Tm*x(4);
sx2=sin(x(2));
cx2=cos(x(2));
aux=(theta*M-alpha*alpha*cx2*cx2);
x(3) = x(3) + Tm*(alpha*alpha*g*sx2*cx2-Fr*theta*x(3)-alpha*C*cx2*x(4)-alpha*sx2*x(4)*x(4)+theta*u)/aux;
x(4) = x(4) + Tm*(alpha*M*g*sx2-alpha*Fr*cx2*x(3)-M*C*x(4)-alpha*alpha*sx2*cx2*x(4)*x(4)+alpha*cx2*u)/aux;
% simular una inclinacion del pendulo {>no funciona {>limitando la u si
% funciona OK!
% bueno si limitamos mucho la u el sistema no puede controlarse.
% x(2)=0.00000002;
% control del pendulo realimentando la matriz K
% La matriz K se ha calculado sobre el sistema linealizado
u=Ref*Nbar-Km*x;
% limitar la referencia por mucho que se quiera acelerar no se puede
if (u>limitu) u=limitu;
end
if (u<-limitu) u=-limitu;
end
u;
% Ecuacion de salida
% x(1)=pos
% x(2)=theta
x;
p1 = x(1);
plot(p1,0,'.');
hold on;
th = x(2);
p2 = (x(1)-ls*sin(th))+j*(ls*cos(th));
line([-0.5+p1,+0.5+p1],[-0.2,-0.2]);
line([-0.5+p1,+0.5+p1],[0.0,-0.0]);
line([-0.5+p1,-0.5+p1],[0.0,-0.2]);
line([0.5+p1,0.5+p1],[0.0,-0.2]);
line(real([p1,p2]),imag([p1,p2]));
plot(real(p2),imag(p2),'.','markersize',40);
hold on;
grid on;
axis([-5 5 -0.5 2]);
drawnow;
end
close all
clear all