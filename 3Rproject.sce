//______________________________________________________________________________

// 3R robot motion with Scilab
// Program that generates the vertical motion of one 3R robot and calculate joint speed
//______________________________________________________________________________


clear

// Functions used in the program

// -------------Function for the Inverse statics of the 3R robot-------------------

function[theta1,theta2,theta3]= EndEffector2jointPositionPOSE(x,y,gam)

// Adapt the point P to the equations
xe=x-g3*cos(gam)-g2*sin(gam)
ye=y-g3*sin(gam)+g2*cos(gam)

//Find theta2

d=2*L1*L2
f=(xe-g1*cos(gam))^2+(ye-g1*sin(gam))^2-L1^2-L2^2
theta2=acos(f/d)


//Find theta1

A=L1+L2*cos(theta2)
B=L2*sin(theta2)
E=xe-g1*cos(gam)
F=ye-g1*sin(gam)

s1=(F-(B*E/A))/(((B^2)/A)+A)
c1=(E+B*s1)/A
theta1=atan(s1,c1)

//Find theta3

theta3=gam-theta1-theta2

endfunction

//---------- Fuction to calculate the Jacobian matrix of the 3R robot-------------

function[J]= Jacobian(theta1, theta2)

//Joint points

x2=L1*cos(theta1)
y2=L1*sin(theta1)

x3=x2+L2*cos(theta1+theta2)
y3=y2+L2*sin(theta1+theta2)

// Jacobian matrix

J=[0 y2 y3;0 -x2 -x3;1 1 1]

endfunction

//------------ Fuction for the Inverse Kinematics of the 3R robot --------------------

function[w1,w2,w3]=TwistInvEndEffector(vx,vy,w,theta1,theta2)

// Find the Jacobian matrix
[J]= Jacobian(theta1,theta2)

//Inverse of the matrix
Ji=inv(J)

//Twist of the 3R robot
Te=[vx;vy;w]

// Find velocities of the links
Wr=Ji*Te

// Extract the values from the matrix
w1=Wr(1)
w2=Wr(2)
w3=Wr(3)


endfunction


//-----------Fuction for calculate the Joints points of the 3R robot-----------------

function[x2,y2,x3,y3,xe,ye,xep,yep,xp,yp]=PointsJoints(theta1,theta2,theta3)

// Point joint 2
x2=L1*cos(theta1)
y2=L1*sin(theta1)

//Point joint 3
x3=x2+L2*cos(theta2+theta1)
y3=y2+L2*sin(theta2+theta1)

//Point joint End Effector

xe=x3+g1*cos(theta2+theta1+theta3)
ye=y3+g1*sin(theta2+theta1+theta3)

// Point to build the end effector
xep=xe+g2*sin(theta2+theta1+theta3)
yep=ye-g2*cos(theta2+theta1+theta3)

// Point P
xp=xep+g3*cos(theta2+theta1+theta3)
yp=yep+g3*sin(theta2+theta1+theta3)

endfunction


//--------------------------- MAIN PROGRAM--------------------------------------

//inputs parameters

//Links Lenght
L1=0.62
L2=0.57
g1=0.1
g2=0.2
g3=0.3

//Links Mass
ML1=4
ML2=3
MEE=2

// 3R robot TWIST
vx=0
vy=-0.1
w=0

//Point joint 1
x1=0
y1=0

//loop
yi=-0.2 //Start
yf=-0.7 //Goal
ay=-0.01 // meters
yt=yf-yi //meters
loop=yt/ay //iterations

//Initial state P (Start)
x=0.9
y=-0.2
gam=0

// Find angles of the joints for initial state
[theta1,theta2,theta3]=EndEffector2jointPositionPOSE(x,y,gam)

//Find points of joints for initial state
[x2,y2,x3,y3,xe,ye,xep,yep,xp,yp]=PointsJoints(theta1,theta2,theta3)

//loop for calculte theta's and angular velocities of the Joints from Start point to Goal point

// matrix of velocities index
j=0

//loop
for i=0:loop

// Find angles of the joints
[theta1,theta2,theta3]=EndEffector2jointPositionPOSE(x,y,gam)

//Find points of joints
[x2,y2,x3,y3,xe,ye,xep,yep,xp,yp]=PointsJoints(theta1,theta2,theta3)



// plot 3R links and joints motion

rm = gca(); // handle

//delete plot 3R robot
delete(rm.children);

rm.title.text = '3R robot motion';
rm.x_label.text = 'x';
rm.y_label.text = 'y';
rm.axes_visible = ["on","on","off"]
rm.grid = [1,1];
rm.data_bounds=[-0.2,-1;1,0.2];

plot (x1,y1,"or")
plot (x2,y2,"or")
plot (x3,y3,"or")
plot (x,y,"og")
plot([x1 x2],[y1 y2])
plot([x2 x3],[y2 y3])
plot([x3, xe],[y3 ye])
plot([xe xep],[ye yep])
plot([xep xp],[yep yp])


// Find angular velocities

[w1,w2,w3]=TwistInvEndEffector(vx,vy,w,theta1,theta2)

// Keeping values on respective vectors
j=j+1
vw1(j)=w1
vw2(j)=w2
vw3(j)=w3

// advance to next state
y=y+ay

// delay of 600 ms
tic;sleep(600);toc


end


// changing iteration to seconds
tt=yt/vy  //total time
ti=tt/loop //interation
t=[0:ti:tt] //time vector from 0 to total time

//plot velocities

figure('BackgroundColor',[1 1 1]);

plot(t,vw1',"r")
plot(t,vw2',"g")
plot(t,vw3')

vt = gca(); // handle
vt.title.text = 'Joint speed vs Time';
vt.x_label.text = 'Time (s)';
vt.y_label.text = 'Joint speed [rad/s]';
vt.axes_visible = ["on","on","off"]
vt.grid = [1,1];
vt.auto_scale="on";
