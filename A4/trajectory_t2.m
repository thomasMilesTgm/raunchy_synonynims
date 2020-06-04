clc 
close all

syms t a b

e1= 0 == 75*a+10*b;
e2= 0 == 125*a+25*b+(75-45)*2*pi/360;
[a,b]=solve(e1,e2);


a1 = pi/250;
b1 = -3*pi/100;
d1 = pi/6;

a2 = pi/375;
b2 =-pi/50;
d2 = 75*2*pi/360;

q1= a1.*t.^3 + b1.*t.^2 + d1;
dq1= 3*a1.*t.^2 + 2*b1.*t;


q2= a2.*t.^3 + b2.*t.^2 + d2;
dq2= 3*a2.*t.^2 + 2*b2.*t;


t = 0:0.01:5;

q1=subs(q1,t);
dq1=subs(dq1,t);
q2=subs(q2,t);
dq2=subs(dq2,t);


figure();
hold on
plot(t,q1.*360/(2*pi));
plot(t,q2.*360/(2*pi));

grid();
legend('q_1','q_2')

figure();
hold on
plot(t,dq1.*360/(2*pi))
plot(t,dq2.*360/(2*pi))

grid();
legend('dq_1','dq_2')


