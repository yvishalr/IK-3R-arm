
clc


posX=input('Enter X coordinate : ');
posY=input('Enter Y coordinate : ');
phi=input('Enter orientation of end effector : ');
phi=phi*pi/180;

linkLength1=2;
linkLength2=2;
linkLength3=2;

x2=posX - (linkLength3*cos(phi));
y2=posY - (linkLength3*sin(phi));
r2=x2*x2 + y2*y2;

theta2calc=(x2*x2 + y2*y2 - linkLength1*linkLength1 - linkLength2*linkLength2)/(2*linkLength1*linkLength2);
theta2=-acos(theta2calc);

theta1cos=((linkLength1 + linkLength2*cos(theta2))*x2 +linkLength2*y2*sin(theta2))/r2;
theta1sin=((linkLength1 + linkLength2*cos(theta2))*y2 -linkLength2*x2*sin(theta2))/r2;

theta1=atan2(theta1sin,theta1cos);

theta3= phi-theta1-theta2;


angle1=theta1*180/pi
angle2=theta2*180/pi
angle3=theta3*180/pi


%angle1= 135;
%angle2=45;
%angle3=45;

v1=linspace(pi/2,angle1*pi/180,100);
v2=linspace(0*pi/2,angle2*pi/180,100);
v3=linspace(0*pi/2,angle3*pi/180,100);

Z=[-10 10];
plot (Z,10);
axis([-10 10 0 10]);
axis equal;
grid ON;
hold ON;

%plot(posX,posY,'g-o'); 

for i=1:100
    [A1,B1,C1]=transform(v1(i),v2(i),v3(i),linkLength1,linkLength2,linkLength3);
    x= [0 A1(1) B1(1) C1(1)]; 
    y= [0 A1(2) B1(2) C1(2)]; 
    Plot = plot(x,y,'r','Linewidth',3);
    pause(0.025);
    delete(Plot);
end
     plot(x,y,'r','Linewidth',3);
     
     
function [A,B,C] = transform(m,n,p,l1,l2,l3)

 P=[0;0;0;1];
 T1=[cos(m) -sin(m) 0 0;sin(m) cos(m) 0 0;0 0 1 0;0 0 0 1];
 T2=[cos(n) -sin(n) 0 l1;sin(n) cos(n) 0 0;0 0 1 0;0 0 0 1];
 T3=[cos(p) -sin(p) 0 l2;sin(p) cos(p) 0 0;0 0 1 0;0 0 0 1];
 T4=[0 0 0 l3;0 1 0 0;0 0 1 0;0 0 0 1];
 
 A=T1*T2*P;
 B=T1*T2*T3*P;
 C=T1*T2*T3*T4*P;
end
