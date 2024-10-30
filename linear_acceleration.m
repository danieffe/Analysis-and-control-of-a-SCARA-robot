function A=linear_acceleration(P1, P2, s_ddot)

d=norm(P2 - P1);
A = (P2 - P1) .* s_ddot(:)/d;

end 