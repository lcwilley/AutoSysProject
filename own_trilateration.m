function [x,y] = own_trilateration(X1,X2,X3,R)
    x1 = X1(1);
    y1 = X1(2);
    x2 = X2(1);
    y2 = X2(2);
    x3 = X3(1);
    y3 = X3(2);
    r1 = R(1);
    r2 = R(2);
    r3 = R(3);
    A = -2*x1+2*x2;
    B = -2*y1+2*y2;
    C = r1^2-r2^2-x1^2+x2^2-y1^2+y2^2;
    D = -2*x2+2*x3;
    E = -2*y2+2*y3;
    F = r2^2-r2^2-x2^2+x3^2-y2^2+y3^2;
    x = (C*E-F*B)/(E*A-B*D);
    y = (C*D-A*F)/(B*D-A*E);
end