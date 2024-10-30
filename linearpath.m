function L=linearpath(P1, P2, s)

d=norm(P2 - P1);
L = P1 + (P2 - P1) .* s(:)/d;

end 