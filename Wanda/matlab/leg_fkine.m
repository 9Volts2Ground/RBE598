clc; clear all; close all; format compact;

% Symbolic DH parameters
syms L1 L2 L3 real % Leg lengths
syms shoulder knee ankle real % Joint angles


DHs = [ shoulder, 0, L1, pi/2;
        knee,     0, L2, pi;
        ankle,    0, L3, 0];
   
A1 = DHMatrix(DHs(1,1), DHs(1,2), DHs(1,3), DHs(1,4));
A2 = DHMatrix(DHs(2,1), DHs(2,2), DHs(2,3), DHs(2,4));
A3 = DHMatrix(DHs(3,1), DHs(3,2), DHs(3,3), DHs(3,4));


T_shoulder2foot = simplify( A1 * A2 * A3 )


L1 = 1;
L2 = 2;
L3 = 3;

shoulder = 0;
knee = 0;
ankle = pi/2;

DHs = [ shoulder, 0, L1, pi/2;
        knee,     0, L2, pi;
        ankle,    0, L3, 0];

A1 = DHMatrix_numeric(DHs(1,1), DHs(1,2), DHs(1,3), DHs(1,4));
A2 = DHMatrix_numeric(DHs(2,1), DHs(2,2), DHs(2,3), DHs(2,4));
A3 = DHMatrix_numeric(DHs(3,1), DHs(3,2), DHs(3,3), DHs(3,4));


T_shoulder2foot = A1 * A2 * A3




