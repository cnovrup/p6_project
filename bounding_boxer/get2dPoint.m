function u = get2dPoint(K, R, t, X)
    %u = K*[R -R*t]*X;
    %u = u/u(3);
    p1 = -transpose(R)*t+transpose(R)*X(1:3);
    u = K*p1;
    u = u./u(3);
end

