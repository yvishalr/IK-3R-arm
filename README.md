# Forward and Inverse Kinematics of a 3-DOF Robotic Arm

Matlab project to estimate and visualize the Inverse Kinematics for an 3R (planar) Robotic Arm.
These are a series of scripts which were modified to perform various methods to achieve the same goal.

The simulation displays a primitive animation of the arm from the initial pose to the final desired pose.

![Plot Animation](/resources/animation1.gif)
![Plot Animation](/resources/animation2.gif)

Let's break down each script individually.

## geometricMethod.m

This script involves implementing and solving for inverse kinematics using geometric methods.

`posX`, `posY` and `phi` are user inputs which will be the desired final pose of the arm.

`linkLength1, linkLength2, linkLength3` can be changed to modify the link lengths of the arm.

`theta1,theta2,theta3` are the positions (slope angle) of link 1, 2 and 3 respectively.
The geometric method for calculating these slopes was first solved on paper, and then implemented in the following lines.

```Matlab
x2=posX - (linkLength3*cos(phi));
y2=posY - (linkLength3*sin(phi));
r2=x2*x2 + y2*y2;

theta2calc=(x2*x2 + y2*y2 - linkLength1*linkLength1 - linkLength2*linkLength2)/(2*linkLength1*linkLength2);
theta2=-acos(theta2calc);

theta1cos=((linkLength1 + linkLength2*cos(theta2))*x2 +linkLength2*y2*sin(theta2))/r2;
theta1sin=((linkLength1 + linkLength2*cos(theta2))*y2 -linkLength2*x2*sin(theta2))/r2;

theta1=atan2(theta1sin,theta1cos);

theta3= phi-theta1-theta2;
```

Initial pose for the arm is specified as the first argument to the vectors `v1, v2, v3` within the _linspace_ function.
These vectors are used to interpolate a fixed number of points between the intial pose and final pose to create the animation.

The remainder of the script is entirely to plot the animation for the arm.
The `transform` method is implemented using Denavit-Hartenberg parameters.
This provides the forward kinematics for the 3R planar arm to plot every intermediate pose of the arm which in turn generates a smooth animation.

## newtonRaphsonMethod.m

This script makes use of an optimization method, namel **Newton Raphson** method, to arrive at the solution for the inverse kinematics of the arm. Newton Raphson is a form of gradient descent method and is less efficient in comparison to the geometrical method since the solution is arrived at over multiple iterations.

User inputs were not implemented here and modification of `posX` and `posY` must be manually done. Further the third variable for phi (desired angle for end effector) has been removed, hence the coordinate will be reached at any arbitrary angle.

Additionally, initial pose can now be changed by altering the angles mentioned in `restAngle1, restAngle2, restAngle3` respectively.

The proposed method first requires you to know the forward kinematics of the arm to determine it's pose which is specified here:

```Matlab
Xcurrent=linkLength1*cos(theta1)+linkLength2*cos(theta1+theta2)+linkLength3*cos(theta1+theta2+theta3);
Ycurrent=linkLength1*sin(theta1)+linkLength2*sin(theta1+theta2)+linkLength3*sin(theta1+theta2+theta3);

```

A Jacobian matrix is solved for using the following formula:-

![formula](/resources/jacobian.jpg)

and continuous iterations are performed using the following optimization

```Matlab
deltaq=pinv(jacobianMatrix)*error; % pseudo inverse of the jacobian

theta1=theta1+deltaq(1);
theta2=theta2+deltaq(2);
theta3=theta3+deltaq(3);

Xcurrent=linkLength1*cos(theta1)+linkLength2*cos(theta1+theta2)+linkLength3*cos(theta1+theta2+theta3);
Ycurrent=linkLength1*sin(theta1)+linkLength2*sin(theta1+theta2)+linkLength3*sin(theta1+theta2+theta3);

error=[posX;posY]-[Xcurrent;Ycurrent];
```

This procedure is continued until the error is within/under an acceptable range.

The animation is plotted exactly as it was in **geometricalMethod.m**

## newtonRaphsonIterations.m

The solution is exactly the same as specified in **newtonRaphsonMethod.m**.
Only difference between the two scripts is that the animation in this case is created by directly plotting the variation iterations the optimizing equation arrives at.
There is no involvement of any interpolations to produce an animation after finding the solution, each intermediate solution is directly being plotted and hence the animation may not be as smooth.

## estimationWithPhi.m

This script is the same as newtonRaphsonMethod.m except here a third parameter is introduced, i.e, `phi`.
The jacobian is modified accordingly since a third state variable is introduced and must also be solved for.

This script additionally generates a CSV file containing the angles to produce the animation. This was done to produce the same animation on a 3-D model on a seperate project on coppeliaSim (V-REP).
