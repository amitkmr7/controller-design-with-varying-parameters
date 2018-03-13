%% open loop step response
% The TF of a second order system for example be: (1/(s^2+10s+20))

num1 = 12;
den1 = [1 10 20];
sys1 = tf(num1,den1);
step(sys1)

%% proportional controller
% at kp=10
num2=1;
den2=[1 10 20];
kp=10;
[numCL,denCL]=  cloop((kp*num2),den2 ,-1);
sys2=tf(numCL,denCL);
t=0:0.01:2;
step(sys2,t)
%% proportional controller with increased value of kp 
% at kp=500
num3=1;
den3=[1 10 20];
kp=500;
[numCL,denCL]=  cloop((kp*num3),den3 ,-1);
sys3=tf(numCL,denCL);
t=0:0.01:2;
step(sys3,t)
%% derivative controller
num1= 1;
den1=[1 10 20];
kp=500;
kd=100;
numc=[kd kp];
[numCL , denCL]= cloop(conv(numc,num1),den1);
t=0:0.01:2;
step(numCL,denCL,t)
%% integral controller
num1=1;
den1=[1 10 20];
kp=500;
ki=10;
kd=0;
numc=[kd kp ki];
denc=[1 0];
[numCL, denCL]= cloop(conv(num1 ,numc),conv(den1,denc));
t=0:0.1:800;
sys5=tf(numCL, denCL);
step(sys5,t)
axis([0 100 0 1.5])
%% PID Controller
num1=1;
den1=[1 10 20];
kp=500;
ki=1;
kd=100;
numc=[kd kp ki];
denc=[1 0];
[numCL,denCL]=cloop(conv(num1,numc),conv(den1,denc));
t=0:0.1:1000;
sys6=tf(numCL,denCL);
step(sys6,t)

