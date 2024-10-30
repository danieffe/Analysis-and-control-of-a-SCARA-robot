function [s, s_dot, s_ddot]=trapezoidal_anticipated(pi, pf, ti, tf, t_tot, flag_via_point, anticipo)

%valore iniziale e finale
s_i=0; 
s_f=norm(pf-pi);

%variabile tempo
delta_t=tf-ti;
t=linspace(0,delta_t,1000*delta_t);

%fisso i parametri arbitrari attraverso le formule
sc_dot=1.5*abs(s_f-s_i)/delta_t;
tc=(s_i-s_f+sc_dot*delta_t)/sc_dot; 
sc_ddot=(sc_dot^2)/(s_i-s_f+sc_dot*delta_t);

%profilo trapezoidale
for k=1:length(t)
    %tra 0 e tc
    if t(k)<=tc
        s_primo(k)=s_i+(1/2)*sc_ddot*t(k).^2;
        s_primo_dot(k)=sc_ddot*t(k);
        s_primo_ddot(k)=sc_ddot;
    %tra tc e delta_t-tc
    else if (t(k)>tc && t(k)<=delta_t-tc)
            s_primo(k)=s_i+sc_ddot*tc*(t(k)-tc/2);
            s_primo_dot(k)=sc_ddot*tc;
            s_primo_ddot(k)=0;
    %tra delta_t-tc e delta_t
    else if (t(k)>delta_t-tc && t(k)<=delta_t)
            s_primo(k)=s_f-(1/2)*sc_ddot*(delta_t-t(k)).^2;
            s_primo_dot(k)=sc_ddot*(delta_t-t(k));
            s_primo_ddot(k)=-sc_ddot;
        end
      end
    end
end

l=linspace(0,ti,1000*ti);
m=linspace(ti+0.001,tf,1000*delta_t);

if flag_via_point==0

    s(1:length(l))=0;
    s(length(l)+1:length(l)+length(m))=s_primo;
    s(length(l)+length(m)+1:1000*t_tot)=norm(pf-pi);
    
    s_dot(1:length(l))=0;
    s_dot(length(l)+1:length(l)+length(m))=s_primo_dot;
    s_dot(length(l)+length(m)+1:1000*t_tot)=0;
    
    s_ddot(1:length(l))=0;
    s_ddot(length(l)+1:length(l)+length(m))=s_primo_ddot;
    s_ddot(length(l)+length(m)+1:1000*t_tot)=0;

else
    s(1:length(l)-1000*anticipo)=0;
    s(length(l)-1000*anticipo+1:length(l)+length(m)-1000*anticipo)=s_primo;
    s(length(l)+length(m)-1000*anticipo+1:1000*t_tot)=norm(pf-pi);
    
    s_dot(1:length(l)-1000*anticipo)=0;
    s_dot(length(l)-1000*anticipo+1:length(l)+length(m)-1000*anticipo)=s_primo_dot;
    s_dot(length(l)+length(m)-1000*anticipo+1:1000*t_tot)=0;
    
    s_ddot(1:length(l)-1000*anticipo)=0;
    s_ddot(length(l)-1000*anticipo+1:length(l)+length(m)-1000*anticipo)=s_primo_dot;
    s_ddot(length(l)+length(m)-1000*anticipo+1:1000*t_tot)=0;


end 