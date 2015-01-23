clear all
close all
clc

syms r p y

w1 = [0 0 0; 0 0 0; 0 0 -1];
w2 = w1 + [sin(y), 0, 0; cos(y), 0, 0; 0, 0, 0];
w3 = w2 + [0 cos(r)*-cos(y) 0; 0 cos(r)*sin(y) 0; 0 sin(r) 0];

M = simplify(w3);

Minv = simplify(inv(M))
