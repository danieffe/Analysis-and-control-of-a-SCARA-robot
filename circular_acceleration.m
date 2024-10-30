function A=circular_acceleration(P1, P2, s, s_dot, s_ddot)

theta=s/norm(P2-P1)*pi+pi/2;
c= [(P2(1)+P1(1))/2,(P2(2)+P1(2))/2,(P2(3)+P1(3))/2 ];
r = norm(P1-c);

x = -s_dot.^2 .* cos(theta)*1/r - s_ddot.*sin(theta);
y =  s_dot.^2 .* sin(theta)*1/r + s_ddot.*cos(theta);
z = 0*ones(size(x));

A=[x',y',z'];


end 