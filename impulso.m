M = 60;
m = 54;
b = 0.1;
i = 0.006;
g = 9.8;
l = 0.3;

q = (M+m)*(i+m*l^2)-(m*l)^2;   %simplifica entrada

num1 = [m*l/q  0  ];
den1 = [1  b*(i+m*l^2)/q  -(M+m)*m*g*l/q  -b*m*g*l/q  0];

num2 = [(i+m*l^2)/q  0  -m*g*l/q];
den2 = den1

kd = 200;
k = 500;
ki = 1;
numPID = [kd k ki];
denPID = [1 0];

numc = conv(num2,denPID);
denc = polyadd(conv(denPID,den2),conv(numPID,num1));
t=0:0.01:5;
impulse(numc,denc,t)
