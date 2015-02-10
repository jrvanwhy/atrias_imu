clear all
close all
clc

% Roll, pitch, yaw, and thir derivatives
syms r p y dr dp dy

w1 = dp * [0; 1; 0];

w2RotMat = [ 1 0      0
             0 cos(r) -sin(r)
             0 sin(r)  cos(r) ];
w2 = simplify(w2RotMat * w1 + dr * [1; 0; 0]);

w3RotMat = [ sin(y) -cos(y) 0
             cos(y)  sin(y) 0
             0       0      1 ];
w3 = simplify(w3RotMat * w2 - dy * [0; 0; 1]);

% Grab the linear transformation matrix
M = simplify(jacobian(w3, [dr; dp; dy]));

% Invert it
Minv = simplify(inv(M))
