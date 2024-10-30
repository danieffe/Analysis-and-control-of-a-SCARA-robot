close all
clear all 

d0=1;
a1=0.5;
a2=0.5;    
l1=0.25;
l2=0.25;
ml1=20;
ml2=20;
ml3=10;
ml4=0;
Il1=4;
Il2=4;
Il3=0;
Il4=1;
kr1=1;
kr2=1;
kr3=50;
kr4 =20;
Im1=0.01;
Im2=0.01;
Im3=0.005;
Im4=0.001;
Fm1=0.00005;
Fm2=0.00005;
Fm3=0.01;
Fm4 =0.005;
mm1=0;
mm2=0;
mm3=0;
mm4=0;
ml=3; %massa del carico

p = [
    0.3, 0.2, 0, 0;
    0.35, 0.2, 0, pi/12;
    0.4, 0.2, 0, pi/10;
    0.4, 0.2, 0.2, pi/8;
    0.4, 0.2, 0.4, pi/6;
    0.45, 0.2, 0.4, pi/4;
    0.45, 0.15, 0.4, pi/2;
    0.45, 0.1, 0.4, 2*pi/3;
    0.45, 0.05, 0.4, pi/2;
    0.4, 0.05, 0.4, pi/4;
    0.4, 0.05, 0.2, pi/6;
    0.4, 0.05, 0, pi/8;
    0.35, 0.05, 0, pi/10;
    0.3, 0.05, 0, pi/12;
    % Nuovi punti aggiunti
    0.25, 0.05, 0, pi/12; % Punto 1
    0.2, 0.05, 0, pi/12; % Punto 2
    0.2, 0.1, 0, pi/10; % Punto 3
    0.2, 0.15, 0, pi/8; % Punto 4
    0.2,0.17,0,pi/8;
    0.3,0.18,0,pi/8;
    0.2, 0.2, 0, pi/6; % Punto 5
    0.22,0.2,0,pi/4;
    0.25, 0.2, 0, pi/2; % Punto 6
    0.3, 0.2, 0, pi; % Punto 7
    0.35, 0.2, 0, pi;] % Punto 8]
P1=retta(p(1,1:3),p(2,1:3));
P2=retta(p(2,1:3),p(3,1:3));
P3=viapoints(p(3,1:3),p(4,1:3),p(5,1:3),0.6);
P4=retta(p(4,1:3),p(5,1:3));
P5=circon(p(6,1:3),[0.46,0.6,0.4],-pi/20);
P6=retta(p(7,1:3),p(8,1:3));
P7=retta(p(8,1:3),p(9,1:3));
P8=retta(p(9,1:3),p(10,1:3));
P9=retta(p(10,1:3),p(11,1:3));
P10=viapoints(p(11,1:3),p(12,1:3),p(13,1:3),0.8);
P11=retta(p(13,1:3),p(14,1:3));
P12=retta(p(14,1:3),p(15,1:3))
P13=retta(p(15,1:3),p(16,1:3));
P14=retta(p(17,1:3),p(18,1:3));
 P15=retta(p(18,1:3),p(19,1:3));
 P16=retta(p(19,1:3),p(20,1:3));
 P17=viapoints(p(20,1:3),p(21,1:3),p(22,1:3),0.5);
  P18=retta(p(22,1:3),p(23,1:3));
  P19=retta(p(23,1:3),p(24,1:3));
 P20=retta(p(24,1:3),p(25,1:3));


O1=orientamento(p(1,4),p(2,4),length(P1));
O2=orientamento(p(2,4),p(3,4),length(P2));
O3=orientamento(p(3,4),p(4,4),length(P3));
O4=orientamento(p(4,4),p(5,4),length(P4));
O5=orientamento(p(5,4),p(6,4),length(P5));
O6=orientamento(p(6,4),p(7,4),length(P6));
O7=orientamento(p(7,4),p(8,4),length(P7));
O8=orientamento(p(8,4),p(9,4),length(P8));
O9=orientamento(p(9,4),p(10,4),length(P9));
O10=orientamento(p(10,4),p(11,4),length(P10));
O11=orientamento(p(11,4),p(12,4),length(P11));
O12=orientamento(p(12,4),p(13,4),length(P12));
O13=orientamento(p(13,4),p(14,4),length(P13));
O14=orientamento(p(14,4),p(15,4),length(P14));
O15=orientamento(p(15,4),p(16,4),length(P15));
O16=orientamento(p(16,4),p(17,4),length(P16));
 O17=orientamento(p(17,4),p(18,4),length(P17));
 O18=orientamento(p(18,4),p(19,4),length(P18));
  O19=orientamento(p(19,4),p(20,4),length(P19));
  O20=orientamento(p(20,4),p(21,4),length(P20));

Ps=[P1; P2(2:end,:); P3(2:end,:); P4(2:end,:); P5(2:end,:); P6(2:end,:); P7(2:end,:); P8(2:end,:); P9(2:end,:); P10(2:end,:); P11(2:end,:);P12(2:end,:);P13(2:end,:);
    P14(2:end,:); P15(2:end,:); 
P16(2:end,:); P17(2:end,:);
P18(2:end,:); P19(2:end,:); P20(2:end,:);];
 O=[O1; O2(2:end,:); O3(2:end,:); O4(2:end,:); O5(2:end,:); O6(2:end,:); O7(2:end,:); O8(2:end,:); O9(2:end,:); O10(2:end,:); O11(2:end,:); O12(2:end,:); O13(2:end,:);
     O14(2:end,:); O15(2:end,:);
  O16(2:end,:);
  O17(2:end,:); 
  O18(2:end,:); O19(2:end,:);O20(2:end,:);];

P = [Ps; Ps(end,:); Ps(end,:)];
 O = [O; O(end,:); O(end,:)];
figure()
plot3(p(:,1),p(:,2),p(:,3))
hold on
plot3(P(:,1),P(:,2),P(:,3),'r')
legend("traiettoria desiderata","traiettoria effettiva")
 grid on

P = [P; P(end,:); P(end,:)];
O = [O; O(end,:); O(end,:)];

figure
 %viene plottato il percorso P. P è una matrice 
plot3(P(:,1),P(:,2),P(:,3))                           %dove le prime tre colonne rappresentano 
%                            rispettivamente tutti valori nelle dimensioni 
%                            x,y,z
hold on
plot3(p(:,1),p(:,2),p(:,3),'*') %vengono plottati tutti i punti che avevamo
%                                scelto per definire il percorso. Infatti
%                                vengono prelevate le prime 3 colonne
%                                (dimensioni x,y,z) della matrice p. Puoi
%                                notare che tutti i punti di p sono anche
%                                punti di P, ad eccezione dei punti di via,
%esterni alla traiettoria P.
grid on 
axis('equal')
xlabel('x')
ylabel('y')
zlabel('z')
set(gcf,'color','white')%accessori al plot


%%% interpolazione del segmento 
tk = linspace(0,61,24)
y=[ 0,...
     length(P1(:,1)),...
    length(P1(:,1))+length(P2(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1))+length(P13(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1))+length(P13(:,1))+length(P14(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1))+length(P13(:,1))+length(P14(:,1))+length(P15(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1))+length(P13(:,1))+length(P14(:,1))+length(P15(:,1))+length(P16(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1))+length(P13(:,1))+length(P14(:,1))+length(P15(:,1))+length(P16(:,1))+length(P17(:,1)),...
     length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1))+length(P13(:,1))+length(P14(:,1))+length(P15(:,1))+length(P16(:,1))+length(P17(:,1))+length(P18(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1))+length(P13(:,1))+length(P14(:,1))+length(P15(:,1))+length(P16(:,1))+length(P17(:,1))+length(P18(:,1))+length(P19(:,1)),...
    length(P1(:,1))+length(P2(:,1))+length(P3(:,1))+length(P4(:,1))+length(P5(:,1))+length(P6(:,1))+length(P7(:,1))+length(P8(:,1))+length(P9(:,1))+length(P10(:,1))+length(P11(:,1))+length(P12(:,1))+length(P13(:,1))+length(P14(:,1))+length(P15(:,1))+length(P16(:,1))+length(P17(:,1))+length(P18(:,1))+length(P19(:,1))+length(P20(:,1)),...
  length(P1(:,1));]
a=spline(tk,[0,y,0] );%"a=spline(x,y)" ritorna una funzione polinomiale
%                       ottenuta dall'interpolazione dei valori in y 
%                       secondo una spline cubica (vedi libro). Tale
%                       funzione serve anche a usare i comandi ppval e
%                      unmkpp.
%                     "a=spline(tk, [0 y 0]" fa la stessa cosa, ma
%                     evidenzia che la pendenza ai punti estremi deve
%                     essere nulla.


a_dot=a;
a_dot.order=a.order-1;
for i=1:3
    a_dot.coefs(:,i)=a.coefs(:,i)*(4-i);
end

a_dotdot=a_dot;
a_dotdot.order=a_dot.order-1;
for i=1:2
    a_dotdot.coefs(:,i)=a_dot.coefs(:,i)*(3-i);
end

t1=linspace(tk(1),tk(end),length(P(:,1)));

s1=ppval(a, t1);%crea una struttura
s1(s1>length(P(:,1))) = length(P(:,1));
s1(s1<0.01) = 0.01;
s1_dot=ppval(a_dot, t1);
s1_dotdot=ppval(a_dotdot,t1);
%s1_r=cumtrapz(s1_dot).*[0 diff(t1)];%Q = cumtrapz(Y) calcola l'integrale cumulativo 
%approssimativo di Y tramite il metodo trapezoidale con spaziatura
%unitaria, diff fa l'integrale
%s1_rr=cumtrapz(s1_dotdot).*[0 diff(t1)];


% traiettoria [tempo x y z orient]
T=zeros(ceil(length(s1)*1),5);
for i=1:length(s1)
    T(i,:)=[t1(i), interpol(P(:,:),s1(i)), interpol(O(:),s1(i))];
end


Tdot=T;
Tdot(1,2:end)=zeros(1,4);
for i=2:length(T)
    if (s1(i)-s1(i-1)) == 0
        Tdot(i,2:end)=[0 0 0 0];
    else
        Tdot(i,2:end)=(T(i,2:end)-T(i-1,2:end))/(s1(i)-s1(i-1))*s1_dot(i);
    end
end
Tdotsmooth = movmean(Tdot,200,1);%M = movmean(A,[kb kf]) calcola la media con una finestra di lunghezza kb+kf+1 che include 
%l'elemento nella posizione corrente, kb elementi indietro e kf elementi avanti.

Tdotdot=Tdot;
Tdotdot(1,2:end)=zeros(1,4);
for i=2:length(T)
     if (s1_dot(i)-s1_dot(i-1)) == 0
        Tdotdot(i,2:end)=[0 0 0 0];
     else
        %Tdotdot(i,2:end)=(Tdot(i,2:end)-Tdot(i-1,2:end))/(s1_dot(i)-s1_dot(i-1))*s1_dotdot(i);
        Tdotdot(i,2:end)=(Tdotsmooth(i,2:end)-Tdotsmooth(i-1,2:end))/(s1_dot(i)-s1_dot(i-1))*s1_dotdot(i);
    end
end
Tdotdot(end,2:end)=zeros(1,4);

figure
plot(T(:,1),T(:,2:end));
legend('x','y','z','O')
%plot(Tdot(:,1),Tdot(:,2:end));
figure
plot(Tdotsmooth(:,1),Tdotsmooth(:,2:end));
legend('xdot','ydot','zdot','Odot')
hold off
figure
plot(Tdotdot(:,1),Tdotdot(:,2:end));
legend('xdotdot','ydotdot','zdotdot','Odotdot')

%Coefficients
Kd = diag([108 68 58 200]);
Kp = diag([13 13 11 1000]);

%Control parameters
KP = diag([1 1 1 1]*50);
KD = diag([1 1 1 1]*150);
H=[zeros(4) eye(4); -KP -KD];
P=1e3*eye(8);
Q=lyap(H,P);

%Adaptive control
lambda = diag ([1 1 1 1]);
PI0 = [ml1; Il1; Im1; Fm1; ml2; Il2; Im2; Fm2; ml3; Im3; Fm3;ml4; Il4; Im4; Fm4]; %parametri dinamici

%Funzioni
function P = retta(p1, p2)%3 coordinate degli estremi dei 2 punti
    

    d=norm(p2-p1);
    s=linspace(0,1,ceil(100*d));% ceil approssima per eccesso, y = linspace(x1,x2,n) generates n points.

    P=(1-s).*p1'+s.*p2'; 
    %lo posso vedere come p+s(p2-p1)
    P=P';

end


function a = cubicpol(tf,sf)
    %calcola s(t)
    
    A=[1 0 0 0;   %pos
        0 1 0 0;  %vel iniziale
        1 tf tf^2 tf^3; %pos finale
        0 1 2*tf 3*tf^2]; %vel finale
    b=[0 0 sf 0]';
    a=inv(A)*b;
end


function O = orientamento(p1, p2, points)%points lunghezza punto intermedio
    t=linspace(0,1,points);
    a=cubicpol(1,1);
    s=a(4)*t.^3+a(3)*t.^2+a(2)*t+a(1);  
        
    O=(1-s).*p1'+s.*p2';% p1+s(p2-p2)
    O=O';

end

function P = viapoints(pi,pv,pf,dtv)
%calcola percorso da pi a pf passando per pv
    v1=(pv-pi)./1;  %Si suppone inoltre che per passare dal punto iniziale al punto di via e dal punto di via a quello
                                                   %finale il tempo sia di 0.5 s.
    v2=(pf-pv)./1;
    av=(v2-v1)./dtv; % av accelerazione del tratto parabolico, dtv durata acc tratto parabolico
    
    l1=norm(v1)*(1-dtv)/2; %spazio percorso, perchè dtv minore di 1 che è 0.5+0.5, faccio 0.35 qui 0.35 dopo e l'accelerazione
    l2=norm(v2)*(1-dtv)/2;
    lv=norm(v1)*dtv+1/2*norm(av)*dtv^2;%vt+1/2at^2
    
    t1=linspace(0,(1-dtv)/2,ceil(l1*1000))';% il tempo da 0 a 0.35
    tv=linspace(0,dtv,ceil(lv*1000))';%il tempo da 0 a 0.3
    t2=linspace(0,(1-dtv)/2,ceil(l2*1000))';% gli ultimi 0.35
    
    P1=pi+v1.*t1;
    P2=P1(end,:)+v1.*tv+1/2.*av.*tv.^2;%accelerazione
    P3=P2(end,:)+v2.*t2;
    P=[P1; P2(2:end,:); P3(2:end,:)];
end
%pag 188
function P = circon(p1,c,arc)
    %construisce arco di circonferenza da p1 centrata in c
    raggio=sqrt((c(1)-p1(1))^2+(c(2)-p1(2))^2);
    d=abs(raggio*arc); % arco
    fase=atan2((p1(2)-c(2))/raggio,(p1(1)-c(1))/raggio); %calcolo della fase
    s=linspace(0,1,ceil(100*d));
    
    P=zeros(length(s),3);
    P(:,1)=c(1)+raggio*(cos(s*arc-fase)); %costruisco l'arco in maniera incrementale, da meno -3.14 a 0
    P(:,2)=c(2)+raggio*(-sin(s*arc-fase));
    P(:,3)=p1(3);%uguale
end


function yinter = interpol(y,x)
    i=ceil(x);
    if i >1
        yinter=y(i-1,:)+(y(i,:)-y(i-1,:))*(x-i+1);
    else
        yinter=y(1,:);
    end
end