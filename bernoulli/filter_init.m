function [m, P] = filter_init()
m = [0.1;0;0.1;0];
P = diag([100 10 100 10]).^2;