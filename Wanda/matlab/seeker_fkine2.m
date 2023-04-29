clc; clear all; close all; format compact;

syms az el real;
syms neckx necky neckz real;
syms seeker real;
syms pie_half real;

% Transform from neck to seeker axis
Tneck = sym( eye(4) );
Tneck(1,4) = neckx;
Tneck(3,4) = neckz;

Rx = sym( eye(4) );
Rx(1:3,1:3) = rotx( pie_half );

Rz = sym( eye(4) );
Rz(1:3,1:3) = rotz( el );

T_neck2seeker = Tneck * Rx * Rz;
T_neck2seeker = simplify( T_neck2seeker )

