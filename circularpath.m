function C=circularpath(P1, P2,s)

%calcolo parametri circonferenza 
c= [(P2(1)+P1(1))/2,(P2(2)+P1(2))/2,(P2(3)+P1(3))/2 ];

r = norm(P1-c);
theta=s/norm(P2-P1)*pi+pi/2;

% Calcola i punti sulla circonferenza parametrica
x = c(1) - r * cos(theta);
y = c(2) + r * sin(theta);
z = P1(:,3)*ones(size(x));

C=[x',y',z'];

end 