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

T_shoulder2shoulder = eye(4);
T_shoulder2knee = simplify(A1)
T_shoulder2ankle = simplify(T_shoulder2knee*A2)
T_shoulder2foot = simplify(T_shoulder2ankle*A3)

% J = zeros(3,3);
J(:,1) = cross( T_shoulder2shoulder(1:3,3), T_shoulder2foot(1:3,4) - T_shoulder2shoulder(1:3,4) );
J(:,2) = cross( T_shoulder2knee(1:3,3), T_shoulder2foot(1:3,4) - T_shoulder2knee(1:3,4) );
J(:,3) = cross( T_shoulder2ankle(1:3,3), T_shoulder2foot(1:3,4) - T_shoulder2ankle(1:3,4) );

simplify( J )
