function A = DHMatrix_numeric(theta, d, a, alpha)

Arough = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
          sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0       sin(alpha)          cos(alpha)          d;
          0       0           0           1];


if abs(alpha) == pi/2
    Arough(1,2) = 0;
    Arough(2,2) = 0;
    Arough(3,3) = 0;
end

A = Arough;

end