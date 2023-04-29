clc; clear all; close all; format compact;

% Symbolic DH parameters
syms az el real
syms az2el_x az2el_z
syms neck

DHs = [az, az2el_z, az2el_x, pi/2;
       el, 0, neck, 0];
   
A1 = DHMatrix( DHs(1,1), DHs(1,2), DHs(1,3), DHs(1,4) );
A2 = DHMatrix( DHs(2,1), DHs(2,2), DHs(2,3), DHs(2,4) );



        