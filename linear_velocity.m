function V=linear_velocity(P1,P2,s_dot)

d=norm(P2 - P1);
V = (P2 - P1) .* s_dot(:)/d;

end 