close
clear
clc

%Central drive torque calculations for the mechanical couple between the
%backhoe and deposition bucket.

%Input parameters
w1=67;      %Bucket Weight, 17 when empty, 67 full
w2=35;      %Backhoe Weight, 10 when empty, about 35 when full
H3=31.08;    %Central axis horizontal distance from the bucket pivot
V3=11.25;     %Central axis height from the ground
Phi1=38;    %Angle between the line connecting the backhoe couple pivot and the backhoe boom
L23=4.8;    %Length of line connecting the backhoe couple pivot and the central axis
L37=20;   %Backhoe Boom Length
L35=14;     %Backhoe cg arm
Theta2=175; %Angle between the front of the backhoe bucket and the backhoe boom
L12=32.75;   %Length of coupling link
V0=25.5;      %Vertical distance to the bucket pivot from the ground
H0=0;       %Horizontal distance to bucket pivot (Zero point)
Phi2=35;    %Angle between the bucket and the bucket couple lever
L03=sqrt((V0-V3)^2+(H3-H0)^2);  %Length of line connecting bucket pivot and the central axis
L78=11;     %Backhoe Bucket Length
L01=3.5;    %Length of the bucket couple lever
L04=18.9;   %Bucket cg arm

n=1000;

%Angle of elevation for backhoe as an array for full ROM.
Theta1=linspace(0,180,n);

%Below, many different geometries are calculated from the input parameters.
%Force interactions are calculated using these geometries and the input parameters.
%These values are all from the general solution for the mechanical couple
%problem I derived.
for i=1:1:n
    V2(i)=V3-cosd(Theta1(i)+Phi1)*L23;
    H2(i)=H3-sind(Theta1(i)+Phi1)*L23;
    V7(i)=V3-cosd(Theta1(i))*L37;
    H7(i)=H3-sind(Theta1(i))*L37;
    V8(i)=V7(i)+sind(Theta2+Theta1(i)-270)*L78;
    H8(i)=H7(i)-cosd(Theta2+Theta1(i)-270)*L78;
    L02(i)=sqrt((V0-V2(i))^2+(H2(i)-H0)^2);
    Theta3(i)=acosd((L01^2+L02(i)^2-L12^2)/(2*L01*L02(i)));
    Theta4(i)=atand((V0-V2(i))/H2(i));
    Theta5(i)=180-Theta3(i)-Theta4(i);
    V1(i)=V0-sind(Theta5(i))*L01;
    H1(i)=-cosd(Theta5(i))*L01;
    Theta6(i)=180-Theta5(i)-Phi2;
    Norm1(i)=(cosd(Theta6(i))*L04); %Lever arm
    Theta7(i)=acosd((L12^2+L02(i)^2-L01^2)/(2*L12*L02(i)));
    Norm2(i)=L02(i)*sind(Theta7(i)); %Lever arm
    L13(i)=sqrt((V1(i)-V3)^2+(H3-H1(i))^2);
    Theta8(i)=acosd((L13(i)^2+L12^2-L23^2)/(2*L13(i)*L12));
    Norm3(i)=sind(Theta8(i))*L13(i); %Lever arm
    Norm4(i)=sind(Theta1(i))*L35; %Lever arm
    T(i)=(((Norm1(i)*w1*Norm3(i))/Norm2(i))+Norm4(i)*w2)/12; %Central drive torque.
end

%Moving plot illustrates the linkage motion
for i=1:1:n
    plot([H0,H1(i)],[V0,V1(i)]);hold on;
    plot([H1(i),H2(i)],[V1(i),V2(i)]);
    plot([H2(i),H3],[V2(i),V3]);
    plot([H3,H7(i)],[V3,V7(i)]);
    plot([H7(i),H8(i)],[V7(i),V8(i)]);hold off;
    axis equal
    pause(0.0001);
end

%Plot of backhoe angle of elevation vs. torque for acutating the deposition
%bucket.
figure
plot(Theta1,T)
title("Backhoe Angle of Elevation vs. Torque Required to Actuate Bucket")
xlabel("Backhoe Angle of Elevation (degrees)")
ylabel("Central Drive Torque Required (ft-lbs)")