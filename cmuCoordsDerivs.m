clear all
close all
clc

% Roll, pitch, yaw, and thir derivatives
syms r p y dr dp dy

% Robot's angular velocity relative to the pitched frontal plane
w1 = dr * [1; 0; 0];

% Robot's angular velocity relative to the yawed sagittal plane
w2RotMat = [ cos(p)  0 sin(p)
             0       1 0
             -sin(p) 0 cos(p) ];
w2 = simplify(w2RotMat * w1 + dp * [0; 1; 0]);

% Robot's angular velocity relative to the world
w3RotMat = [ cos(y) -sin(y) 0
             sin(y)  cos(y) 0
             0       0      1 ];
w3 = simplify(w3RotMat * w2 + dy * [0; 0; 1]);

% Grab the linear transformation matrix
M = simplify(jacobian(w3, [dy; dp; dr]));

% Invert it
Minv = simplify(inv(M))
