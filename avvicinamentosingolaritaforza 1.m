% proggetto do for inserimento dei dati 
% nota nell' analisi dell' andamento dell ellissoide di forza si può
% facilmente notare che partendo dalla configurazione 1  se ci allonteaniamo
% dalla singolarità la dimensione dell' ellissoide aumenta mentre se ci
% avviciniamo alla singolarita coe nelle configurazioni 3 4 la sua
% dimensione tende a diminuire sino a degenerare in una reta nella
% singolarità. 
close
clear all 
clc
d0=1;
a1=0.5;
a2=0.5;
l1=0.25;
l2=0.25;
m1=20
m2=20;
m3=10;
Il1=4;
Il2=4;
Il4=1;
k1=1;
k2=1;
k3=50;
k4=20;
Im1=0.01;
Im2=0.01;
Im3=0.005;
Im4=0.001;
F1=0.00005;
F2=0.00005;
F3=0.01;
F4=0.005;


a=[a1 a2];


teta=[pi/3 -pi/2];

 % guardando la struttura del robot mi accorgo subito che la struttura
 % portante può essere paragonata a quella di un manipolatore a due bracci
 % noto infatti che il ginto prismatico permette uno sliding verticale del
 % 3 braccio mentre il giunto 4 una rotazione attorno ad esso. 

centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));
%deriviamo
J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
(a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (+a(2)*(cos(teta(1)+teta(2))))];
%forza
[V, D]=eig(((J)*(J)')^-1);
maniplobilitymesurement=sqrt(det((J)*(J')))
%V-> autovettore
%D-> autovalori
[x, y]=ellipse(0.5*sqrt((D(1,1))/norm(D))*V(:,1)/norm(V(:,1)),0.5*sqrt((D(2,2)/norm(D)))*V(:,2)/norm(V(:,2)),[centro_x centro_y]);
plot(x, y)
% EQUAZIONI BRACCI
%braccio1
braccio1_x=0;
braccio1_y=0;
braccio1_x1=a(1)*cos(teta(1));
braccio1_y1=a(1)*sin(teta(1));
braccio1x=[braccio1_x,braccio1_x1];
braccio1y=[braccio1_y,braccio1_y1];
%braccio2
braccio2_x1=a(1)*cos(teta(1))+a(2)*cos(teta(1)+teta(2));
braccio2_y1=a(1)*sin(teta(1))+a(2)*sin(teta(1)+teta(2));
braccio2x=[braccio1_x1,braccio2_x1];
braccio2y=[braccio1_y1,braccio2_y1];
hold on
plot(braccio1x,braccio1y,'.-k','markersize',15,'HandleVisibility','off')
hold on
plot(braccio2x,braccio2y,'.-k','markersize',15,'HandleVisibility','off')
hold on

%------------------------------------------------------------------------
teta=[pi/6 -pi/4];
J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
(a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (+a(2)*(cos(teta(1)+teta(2))))];
centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));
[V, D]=eig(((J)*(J)')^-1);
maniplobilitymesurement=sqrt(det((J)*(J')))
%V-> autovettore
%D-> autovalori
[x, y]=ellipse(0.5*sqrt(D(1,1)/norm(D))*V(:,1)/norm(V(:,1)),0.5*sqrt(D(2,2)/norm(D))*V(:,2)/norm(V(:,2)),[centro_x centro_y]);
plot(x, y)
% EQUAZIONI BRACCI
%braccio1
braccio1_x=0;
braccio1_y=0;
braccio1_x1=a(1)*cos(teta(1));
braccio1_y1=a(1)*sin(teta(1));
braccio1x=[braccio1_x,braccio1_x1];
braccio1y=[braccio1_y,braccio1_y1];
%braccio2
braccio2_x1=a(1)*cos(teta(1))+a(2)*cos(teta(1)+teta(2));
braccio2_y1=a(1)*sin(teta(1))+a(2)*sin(teta(1)+teta(2));
braccio2x=[braccio1_x1,braccio2_x1];
braccio2y=[braccio1_y1,braccio2_y1];
hold on
plot(braccio1x,braccio1y,'.-k','markersize',15,'HandleVisibility','off')
hold on
plot(braccio2x,braccio2y,'.-k','markersize',15,'HandleVisibility','off')
hold on
%---------------------------------------------------------------------
teta=[pi/8 -pi/4];
J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
(a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (+a(2)*(cos(teta(1)+teta(2))))];
centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));
[V, D]=eig(((J)*(J)')^-1);
maniplobilitymesurement=sqrt(det((J)*(J')))
%V-> autovettore
%D-> autovalori
[x, y]=ellipse(0.5*sqrt(D(1,1)/norm(D))*V(:,1)/norm(V(:,1)),0.5*sqrt(D(2,2)/norm(D))*V(:,2)/norm(V(:,2)),[centro_x centro_y]);
plot(x, y)
% EQUAZIONI BRACCI
%braccio1
braccio1_x=0;
braccio1_y=0;
braccio1_x1=a(1)*cos(teta(1));
braccio1_y1=a(1)*sin(teta(1));
braccio1x=[braccio1_x,braccio1_x1];
braccio1y=[braccio1_y,braccio1_y1];
%braccio2
braccio2_x1=a(1)*cos(teta(1))+a(2)*cos(teta(1)+teta(2));
braccio2_y1=a(1)*sin(teta(1))+a(2)*sin(teta(1)+teta(2));
braccio2x=[braccio1_x1,braccio2_x1];
braccio2y=[braccio1_y1,braccio2_y1];
hold on
plot(braccio1x,braccio1y,'.-k','markersize',15,'HandleVisibility','off')
hold on
plot(braccio2x,braccio2y,'.-k','markersize',15,'HandleVisibility','off')
hold on

%----------------------------------------------------------------------
teta=[pi/12 -pi/9];
J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
(a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (+a(2)*(cos(teta(1)+teta(2))))];
centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));
[V, D]=eig(((J)*(J)')^-1);
maniplobilitymesurement=sqrt(det((J)*(J')))
%V-> autovettore
%D-> autovalori
[x, y]=ellipse(0.5*sqrt((D(1,1))/norm(D))*V(:,1)/norm(V(:,1)),0.5*sqrt((D(2,2)/norm(D)))*V(:,2)/norm(V(:,2)),[centro_x centro_y]);
plot(x, y)

% EQUAZIONI BRACCI
%braccio1
braccio1_x=0;
braccio1_y=0;
braccio1_x1=a(1)*cos(teta(1));
braccio1_y1=a(1)*sin(teta(1));
braccio1x=[braccio1_x,braccio1_x1];
braccio1y=[braccio1_y,braccio1_y1];
%braccio2
braccio2_x1=a(1)*cos(teta(1))+a(2)*cos(teta(1)+teta(2));
braccio2_y1=a(1)*sin(teta(1))+a(2)*sin(teta(1)+teta(2));
braccio2x=[braccio1_x1,braccio2_x1];
braccio2y=[braccio1_y1,braccio2_y1];
hold on
plot(braccio1x,braccio1y,'.-k','markersize',15,'HandleVisibility','off')
hold on
plot(braccio2x,braccio2y,'.-k','markersize',15,'HandleVisibility','off')
hold on
%----------------------------------------------------------------------

legend("Configurazione 2","Configurazione 1","Configurazione 3","Configurazione 4")
% EQUAZIONI BRACCI
%braccio1

%----------------------------------------------------------------------
grid on
axis('equal')
title('Ellisoide in forza')
xlabel('[m]'),ylabel('[m]')
set(gcf,'color','white')

function [x, y] = ellipse(a, b, c)

theta=linspace(0,2*pi);
%sezione piana di un ellissoide
x=c(1)+b(1)*sin(theta)+a(1)*cos(theta);
y=c(2)+b(2)*sin(theta)+a(2)*cos(theta);

end
