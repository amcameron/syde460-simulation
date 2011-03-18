function x = localize_state(X)
    Rb       = R(X(7), X(8), X(9));
    x(1:3)   = Rb * X(1:3);
    x(4:6)   = Rb * X(4:6);
    x(7:9)   = -X(7:9);
    x(10:12) = Rb * X(10:12);
end

