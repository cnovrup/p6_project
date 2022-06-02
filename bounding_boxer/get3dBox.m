function [p1,p2,p3,p4,p5,p6,p7,p8] = get3dBox(X,w,h)
    x = X(1);
    y = X(2);
    z = X(3);
    p1 = [x-w/2;y-w/2;z-h/2];
    p2 = [x+w/2;y-w/2;z-h/2];
    p3 = [x-w/2;y+w/2;z-h/2];
    p4 = [x+w/2;y+w/2;z-h/2];

    p5 = [x-w/2;y-w/2;z+h/2];
    p6 = [x+w/2;y-w/2;z+h/2];
    p7 = [x-w/2;y+w/2;z+h/2];
    p8 = [x+w/2;y+w/2;z+h/2];
end

