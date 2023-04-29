clc; clear all; close all; format compact;

syms coxa femur tibia real;
syms q1 q2 q3 real;
syms pi_half Pie real;

% Get transform from shoulder to coxa
T_shoulder2coxa = sym( eye(4) );
T_shoulder2coxa(1:3,1:3) = rotz( q1 );


% Get transform from coxa to femur
Tcoxa = sym( eye(4) );
Tcoxa(1,4) = coxa;

Rx = sym( eye(4) );
Rx(1:3,1:3) = rotx( pi_half );

Rq2 = sym( eye(4) );
Rq2(1:3,1:3) = rotz( q2 );

T_coxa2femur = Tcoxa * Rx * Rq2;
T_coxa2femur = simplify( T_coxa2femur )

% Get transform from femur to tibia
Tfemur = sym( eye(4) );
Tfemur(1,4) = femur;

Rx = sym( eye(4) );
Rx(1:3,1:3) = rotx( Pie );

Rq3 = sym( eye(4) );
Rq3(1:3,1:3) = rotz( q3 );

T_femur2tibia = Tfemur * Rx * Rq3;
T_femur2tibia = simplify( T_femur2tibia )

% Get transform from tibia to foot
T_tibia2foot = sym( eye(4) );
T_tibia2foot(1,4) = tibia;

simplify( T_shoulder2coxa * T_coxa2femur * T_femur2tibia * T_tibia2foot )
