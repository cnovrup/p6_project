function T = base2camera(q1, q2, q3, q4, q5, q6)
     param = [pi/2 0 128.3+115 q1;
             pi 280 30 q2+pi/2;
             pi/2 0 20 q3+pi/2;
             pi/2 0 140+105 q4+pi/2;
             pi/2 0 28.5+28.5 q5+pi;
             0 0 105+130 q6+pi/2];

     A1 = Amatrix(param(1,:));
     A2 = Amatrix(param(2,:));
     A3 = Amatrix(param(3,:));
     A4 = Amatrix(param(4,:));
     A5 = Amatrix(param(5,:));
     A6 = Amatrix(param(6,:));
    Ac = [0 1 0 -85;
          -1 0 0 0;
          0 0 1 -110;
          0 0 0 1];
    T = (A1*A2*A3*A4*A5*A6);
    %T = Ac*A6*A5*A4*A3*A2*A1;
end

