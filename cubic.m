function [s, s_dot, s_ddot] = cubic(pi, pf, ti, tf, t_tot)

%impongo i valori iniziali e finali per l'arclength
s_i=0; 
s_f=norm(pf-pi);

%variabile tempo
delta_t=tf-ti;
t=linspace(0,delta_t,1000*delta_t);

%coefficienti del polinomio
a0=s_i;
a1=0;
a2=3*(s_f-s_i)/delta_t^2;
a3=-2*(s_f-s_i)/delta_t^3;

%andamento cubico s'
s_primo=a3*t.^3+a2*t.^2+a1*t+a0;
s_primo_dot=3*a3*t.^2+2*a2*t+a1;
s_primo_ddot=6*a3*t+2*a2;

l=linspace(0,ti,1000*ti);
m=linspace(ti+0.001,tf,1000*delta_t);

s(1:length(l))=0;
s(length(l)+1:length(l)+length(m))=s_primo;
s(length(l)+length(m)+1:1000*t_tot)=norm(pf-pi);

s_dot(1:length(l))=0;
s_dot(length(l)+1:length(l)+length(m))=s_primo_dot;
s_dot(length(l)+length(m)+1:1000*t_tot)=0;

s_ddot(1:length(l))=0;
s_ddot(length(l)+1:length(l)+length(m))=s_primo_ddot;
s_ddot(length(l)+length(m)+1:1000*t_tot)=0;

end 