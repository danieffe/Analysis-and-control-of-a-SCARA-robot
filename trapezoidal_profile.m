function [s, s_dot, s_dotdot]=trapezoidal_profile(pi,pf,ti,tf,t_tot,is_viapoint,delta)

s_i=0;
s_f=norm(pf-pi);
dt=tf-ti;

s_c_dot=1.5*abs(s_f-s_i)/dt;
tc=(s_i-s_f+s_c_dot*dt)/s_c_dot;
s_c_dotdot=s_c_dot/tc;


t=linspace(0,dt,1000*dt);

for k=1:length(t)
    if t(k)<=tc
        s_primo(k)=s_i+(1/2)*s_c_dotdot*t(k).^2;
        s_dot_primo(k)=s_c_dotdot*t(k);
        s_dotdot_primo(k)=s_c_dotdot;
    else if (t(k)>tc && t(k)<=dt-tc)
            s_primo(k)=s_i+s_c_dotdot*tc*(t(k)-tc/2);
            s_dot_primo(k)=s_c_dotdot*tc;
            s_dotdot_primo(k)=0;
    else if (t(k)>dt-tc && t(k)<=dt)
            s_primo(k)=s_f-(1/2)*s_c_dotdot*(dt-t(k)).^2;
            s_dot_primo(k)=s_c_dotdot*(dt-t(k));
            s_dotdot_primo(k)=-s_c_dotdot;
        end
        end
    end
end

l=linspace(0,ti,1000*ti);
m=linspace(ti+0.001,tf,1000*dt);

if is_viapoint==0
    s(1:length(l))=0;
    s(length(l)+1:length(l)+length(m))=s_primo;
    s(length(l)+length(m)+1:1000*t_tot)=norm(pf-pi);

    s_dot(1:length(l))=0;
    s_dot(length(l)+1:length(l)+length(m))=s_dot_primo;
    s_dot(length(l)+length(m)+1:1000*t_tot)=0;

    s_dotdot(1:length(l))=0;
    s_dotdot(length(l)+1:length(l)+length(m))=s_dotdot_primo;
    s_dotdot(length(l)+length(m)+1:1000*t_tot)=0;
    
else
    s(1:length(l)-1000*delta)=0;
    s(length(l)-1000*delta+1:length(l)+length(m)-1000*delta)=s_primo;
    s(length(l)+length(m)-1000*delta+1:1000*t_tot)=norm(pf-pi);

    s_dot(1:length(l)-1000*delta)=0;
    s_dot(length(l)-1000*delta+1:length(l)+length(m)-1000*delta)=s_dot_primo;
    s_dot(length(l)+length(m)-1000*delta+1:1000*t_tot)=0;

    s_dotdot(1:length(l)-1000*delta)=0;
    s_dotdot(length(l)-1000*delta+1:length(l)+length(m)-1000*delta)=s_dotdot_primo;
    s_dotdot(length(l)+length(m)-1000*delta+1:1000*t_tot)=0;

end