clear all
close all
clc

%% Lectura de distancias
prompt = 'Introduza la longitud del primer eslabon (l1): ';
l1 = input(prompt);
prompt = 'Introduza la longitud del primer eslabon (l2): ';
l2 = input(prompt);
prompt = 'Introduza la longitud del primer eslabon (l3): ';
l3 = input(prompt);

%% Lectura de ángulos
prompt = 'Introduza la longitud del primer eslabon en grados (theta_1): ';
theta_1 = input(prompt);

prompt = 'Introduza la longitud del primer eslabon en grados (theta_2): ';
theta_2 = input(prompt);

prompt = 'Introduza la longitud del primer eslabon en grados (theta_3): ';
theta_3 = input(prompt);


%% Conversión de grados a radianes
theta_1_rad = deg2rad(theta_1);
theta_2_rad = deg2rad(theta_2);
theta_3_rad = deg2rad(theta_3);

%% Definición de los parámetros de Denavit-Hartenberg
d1 = l1;
d2 = 0;
d3 = 0;

a1 = 0;
a2 = l2;
a3 = l3;

alpha_1 = 90;
alpha_2 = 0;
alpha_3 = 0;
alpha_1_rad = deg2rad(alpha_1);
alpha_2_rad = deg2rad(alpha_2);
alpha_3_rad = deg2rad(alpha_3);

%% Definición del punto inicial del robot
p1 = [0,0,0];

if theta_1_rad>=0
    angVec = 0:0.01:theta_1_rad;
else
    angVec = 0:-0.01:theta_1_rad;
end

for i=1:length(angVec)
    clf
    printAxis();
    grid on 
    Rotz = [cos(angVec(i)) -sin(angVec(i))  0; sin(angVec(i)) cos(angVec(i)) 0; 0 0 1];
    A1 = dhParameters(angVec(i),d1,a1,alpha_1_rad);
    A2 = dhParameters(0,d2,a2,alpha_2_rad);
    A3 = dhParameters(0,d3,a3,alpha_3_rad);
    
    A12 = A1*A2;
    A123 = A1*A2*A3; 

    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    view(30,30);
    pause(0.01);
end
    pause(1);
    
if theta_2_rad>=0
    angVec = 0:0.01:theta_2_rad;
else
    angVec = 0:-0.01:theta_2_rad;
end

for i=1:length(angVec)
    clf
    printAxis();
    grid on 
    Rotz = [cos(theta_1_rad) -sin(theta_1_rad)  0; sin(theta_1_rad) cos(theta_1_rad) 0; 0 0 1];
    A1 = dhParameters(theta_1_rad,d1,a1,alpha_1_rad);
    A2 = dhParameters(angVec(i),d2,a2,alpha_2_rad);
    A3 = dhParameters(0,d3,a3,alpha_3_rad);
    
    A12 = A1*A2;
    A123 = A1*A2*A3; 

    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    view(30,30);
    pause(0.01);
end

pause(1);

if theta_3_rad>=0
    angVec = 0:0.01:theta_3_rad;
else
    angVec = 0:-0.01:theta_3_rad;
end

for i=1:length(angVec)
    clf
    printAxis();
    grid on 
    Rotz = [cos(theta_1_rad) -sin(theta_1_rad)  0; sin(theta_1_rad) cos(theta_1_rad) 0; 0 0 1];
    A1 = dhParameters(theta_1_rad,d1,a1,alpha_1_rad);
    A2 = dhParameters(theta_2_rad,d2,a2,alpha_2_rad);
    A3 = dhParameters(angVec(i),d3,a3,alpha_3_rad);
    
    A12 = A1*A2;
    A123 = A1*A2*A3; 

    p1 = [0 0 0]';
    p2 = A1(1:3,4);
    p3 = A12(1:3,4);
    p4 = A123(1:3,4);
    
    printLink(p1,p2);
    printLink(p2,p3);
    printLink(p3,p4);
    printMiniAxes(p1,Rotz);
    printMiniAxes(p2,A12);
    printMiniAxes(p3,A123);
    printMiniAxes(p4,A123);
    view(30,30);
    pause(0.01);
end
