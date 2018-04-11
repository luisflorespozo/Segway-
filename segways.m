M = 20;
m = 54;
b = 0.1;
i = 0;
g = 9.8;
l = 0.3;

q = (M+m)*(i+m*l^2)-(m*l)^2;   %simplifica entrada

num = [m*l/q  0]
den = [1  b*(i+m*l^2)/q  -(M+m)*m*g*l/q  -b*m*g*l/q]
t=0:0.01:5;
impulse(num,den,t)
axis([0 1 0 60])