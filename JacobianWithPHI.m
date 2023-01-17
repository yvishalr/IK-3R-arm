
clc

posX=-4;
posY=6;
phi=135*pi/180;


radius=sqrt(posX*posX+posY*posY);

restAngle1=pi/2;
restAngle2=-pi/2;
restAngle3=-pi/2;

linkLength1=4;
linkLength2=3;
linkLength3=2;

if radius < (linkLength1+linkLength2+linkLength3)

theta1=restAngle1;
theta2=restAngle2;
theta3=restAngle3;

Xcurrent=linkLength1*cos(theta1)+linkLength2*cos(theta1+theta2)+linkLength3*cos(theta1+theta2+theta3);
Ycurrent=linkLength1*sin(theta1)+linkLength2*sin(theta1+theta2)+linkLength3*sin(theta1+theta2+theta3);

error=[posX;posY;phi]-[Xcurrent;Ycurrent;theta1+theta2+theta3];
iterationCount=0;

while norm(error)>10^-4
    
    jacobianMatrix=[-linkLength1*sin(theta1)-linkLength2*sin(theta1+theta2)-linkLength3*sin(theta1+theta2+theta3) ...
        -linkLength2*sin(theta1+theta2)-linkLength3*sin(theta1+theta2+theta3) ...
        -linkLength3*sin(theta1+theta2+theta3);
        linkLength1*cos(theta1)+linkLength2*cos(theta1+theta2)+linkLength3*cos(theta1+theta2+theta3) ...
        linkLength2*cos(theta1+theta2)+linkLength3*cos(theta1+theta2+theta3) ...
        linkLength3*cos(theta1+theta2+theta3);
        1 1 1];
    
        
        deltaq=pinv(jacobianMatrix)*error;
        
        theta1=theta1+deltaq(1);
        theta2=theta2+deltaq(2);
        theta3=theta3+deltaq(3);
        
        Xcurrent=linkLength1*cos(theta1)+linkLength2*cos(theta1+theta2)+linkLength3*cos(theta1+theta2+theta3);
        Ycurrent=linkLength1*sin(theta1)+linkLength2*sin(theta1+theta2)+linkLength3*sin(theta1+theta2+theta3);

        error=[posX;posY;phi]-[Xcurrent;Ycurrent;theta1+theta2+theta3];
        
        iterationCount=iterationCount+1
        
        if iterationCount > 100000
            break;
        end
end

else
    if(posX==0)
        theta1=pi/2;
        theta2=0;
        theta3=0;
    else
        theta1=atan2(posY,posX);
        theta2=0;
        theta3=0;
    end
end

angle1=wrapToPi(theta1);
angle2=wrapToPi(theta2);
angle3=wrapToPi(theta3);

v1=linspace(restAngle1,angle1,100);
v2=linspace(restAngle2,angle2,100);
v3=linspace(restAngle3,angle3,100);

Z=[-10 10];
plot (Z,10);
axis([-10 10 0 10]);
grid ON;
hold ON;

%plot(posX,posY,'g-o'); 

a=zeros(100,3);

for i=1:100
    [A1,B1,C1]=transform(v1(i),v2(i),v3(i),linkLength1,linkLength2,linkLength3);
    x= [0 A1(1) B1(1) C1(1)]; 
    y= [0 A1(2) B1(2) C1(2)]; 
    Plot = plot(x,y,'r','Linewidth',3);
    pause(0.025);
    delete(Plot);
    
    finalMatrix=[v1(i) v2(i) v3(i)];
    a(i,1:3)=finalMatrix;


end

     plot(x,y,'r','Linewidth',3);
     writematrix(a,'finalMatrix.csv');
     
     
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