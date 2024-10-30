function V=circular_velocity(P1, P2, s, s_dot)

theta=s/norm(P2-P1)*pi+pi/2;

x = - s_dot .* cos(theta);
y = s_dot .* sin(theta);
z = 0*ones(size(x));

V=[x',y',z'];

end 