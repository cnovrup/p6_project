function A = Amatrix(params)%Amatrix(di, theta, ai, alpha)
alpha = params(1);
ai = params(2);
di = params(3);
theta = params(4);

Tzd = [1 0 0 0;
       0 1 0 0;
       0 0 1 di;
       0 0 0 1];
   
Tzth = [cos(theta) -sin(theta) 0 0;
        sin(theta) cos(theta) 0 0;
        0 0 1 0;
        0 0 0 1];
    
Txa = [1 0 0 ai;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
   
Txal = [1 0 0 0;
        0 cos(alpha) -sin(alpha) 0;
        0 sin(alpha) cos(alpha) 0;
        0 0 0 1];
    
A = Tzd * Tzth * Txa * Txal;
end

