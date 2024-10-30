clc
clear all
close all

a=[0.5 0.5];
%%nota che le velocita nella configurazione 4 degenerano ovvero aumentano
%%troppo non sono esplicabili dagli attuatori in quanto siamo in prossimita
%%di una singolarita per cui l' inversa dello jacobbiano ha un determinante
%%prossimo allo zero pari  0.15 e dalla misura di manipolabilita si vede
%%che la configuazione non è indicata in quanto w=0.08 (sta nel file )
%teta=[pi/3 -pi/2]; % Ellissoide 1)
%teta=[pi/6 -pi/4]; % Ellissoide 2)
%teta=[pi/8 -pi/4]; % Elissoide 3)
%teta=[pi/12 -pi/9]; % Elissoide 4)

teta=[pi/3 -pi/2] %questo è a gomito basso


centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));

J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
     (a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (a(2)*(cos(teta(1)+teta(2))))];
 
 maniplobilitymesurement=sqrt(det((J)*(J')))
%forza
 [V,G]=eig(J*J');
 %velocità
 [U,D]=eig((J*J')^-1);

 %autovettori
 vett1_x=V(1,1);
 vett1_y=V(2,1);
 autovettore1_x=[vett1_x 0];
 autovettore1_y=[vett1_y 0];
 m=U(2,1)/U(1,1);
 k=U(2,2)/U(1,2);
 b=linspace(-0.5,2);
 n=centro_y+m*(b-centro_x);
 c=centro_y+k*(b-centro_x);
 
 
 vett2_x=V(1,2);
 vett2_y=V(2,2);
 autovettore2_x=[vett2_x 0];
 autovettore2_y=[vett2_y 0];
 
 % EQUAZIONI BRACCI
          %braccio1
 braccio1_x=0;
 braccio1_y=0;
 braccio1_x1=a(1)*cos(teta(1));
 braccio1_y1=a(1)*sin(teta(1));
 braccio1x=[braccio1_x,braccio1_x1];
 braccio1y=[braccio1_y,braccio1_y1];
         %braccio2 
 braccio2_x1=a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
 braccio2_y1=a(1)*sin(teta(1))+a(2)*(sin(teta(1)+teta(2)));
 braccio2x=[braccio1_x1,braccio2_x1];
 braccio2y=[braccio1_y1,braccio2_y1];
 
 
 
 % EQUAZIONE ELLISSE FORZA
 
 s1=G(1,1)*V(:,1)/norm(V(:,1)/norm(G));
 s2=G(2,2)*V(:,2)/norm(V(:,2)/norm(G));
 beta = linspace(0,2*pi);
 x = (s1(1))*cos(beta)+s2(1)*sin(beta);
 y = (s1(2))*cos(beta)+s2(2)*sin(beta);
          %centro
 x = x + centro_x;
 y = y + centro_y;
 
 
 
 
 s11=(D(1,1)*U(:,1)/norm(U(:,1)/norm(D)))/1000;
 s22=(D(2,2)*U(:,2)/norm(U(:,2)/norm(D)))/1000;
 x1 = s11(1)*cos(beta)+s22(1)*sin(beta);
 y1 = s11(2)*cos(beta)+s22(2)*sin(beta);
          %centro
 x1 = x1 + centro_x;
 y1 = y1 + centro_y; 
figure ()
plot(x1,y1,'-b','Linewidth',1.5)
 hold on
  plot(braccio1x,braccio1y,'.-k','markersize',15,'HandleVisibility','off')
  hold on
  plot(braccio2x,braccio2y,'.-k','markersize',15,'HandleVisibility','off')
  hold on
%%%%%%%%%%%%%%%%%%%%%%%%
teta=[pi/6 -pi/4]
centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));

J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
     (a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (a(2)*(cos(teta(1)+teta(2))))];

maniplobilitymesurement=sqrt(det((J)*(J')))
 
%forza
 [V,G]=eig(J*J');
 %velocità
 [U,D]=eig((J*J')^-1);

 %autovettori
 vett1_x=V(1,1);
 vett1_y=V(2,1);
 autovettore1_x=[vett1_x 0];
 autovettore1_y=[vett1_y 0];
 m=U(2,1)/U(1,1);
 k=U(2,2)/U(1,2);
 b=linspace(-0.5,2);
 n=centro_y+m*(b-centro_x);
 c=centro_y+k*(b-centro_x);
 
 
 vett2_x=V(1,2);
 vett2_y=V(2,2);
 autovettore2_x=[vett2_x 0];
 autovettore2_y=[vett2_y 0];
 
 % EQUAZIONI BRACCI
          %braccio1
 braccio1_x=0;
 braccio1_y=0;
 braccio1_x1=a(1)*cos(teta(1));
 braccio1_y1=a(1)*sin(teta(1));
 braccio1x=[braccio1_x,braccio1_x1];
 braccio1y=[braccio1_y,braccio1_y1];
         %braccio2 
 braccio2_x1=a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
 braccio2_y1=a(1)*sin(teta(1))+a(2)*(sin(teta(1)+teta(2)));
 braccio2x=[braccio1_x1,braccio2_x1];
 braccio2y=[braccio1_y1,braccio2_y1];
 
 
%  q_punto= [J(1,1); J(2,1)];
%  v=J*q_punto;
%  v_x=v(1);
%  v_y=v(2);
%  Vxy=v'*(inv(J*J'))*v;
 
 % EQUAZIONE ELLISSE FORZA
 
 s1=G(1,1)*V(:,1)/norm(V(:,1)/norm(G));
 s2=G(2,2)*V(:,2)/norm(V(:,2)/norm(G));
 beta = linspace(0,2*pi);
 x = (s1(1))*cos(beta)+s2(1)*sin(beta);
 y = (s1(2))*cos(beta)+s2(2)*sin(beta);
          %centro
 x = x + centro_x;
 y = y + centro_y;
 
 
 %EQUAZIONE ELLISSE VELOCITA'
 %(D(1,1)*V(:,1)/norm(V(:,1))/norm(D),D(2,2)*V(:,2)/norm(V(:,2))/norm(D)
 
 
 s11=(D(1,1)*U(:,1)/norm(U(:,1)/norm(D)))/1000;
 s22=(D(2,2)*U(:,2)/norm(U(:,2)/norm(D)))/1000;
 x1 = s11(1)*cos(beta)+s22(1)*sin(beta);
 y1 = s11(2)*cos(beta)+s22(2)*sin(beta);
          %centro
 x1 = x1 + centro_x;
 y1 = y1 + centro_y; 
 plot(x1,y1,'-b','Linewidth',1.5,'Color','r') 
 hold on
  plot(braccio1x,braccio1y,'.-k','markersize',15,'HandleVisibility','on')
  hold on
  plot(braccio2x,braccio2y,'.-k','markersize',15,'HandleVisibility','on')
  hold on 

 %%%%%%%%%%%%%%%%%%%%%%%%%%
  teta=[pi/8 -pi/4]
centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));

J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
     (a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (a(2)*(cos(teta(1)+teta(2))))];
 
 maniplobilitymesurement=sqrt(det((J)*(J')))
%forza
 [V,G]=eig(J*J');
 %velocità
 [U,D]=eig((J*J')^-1);

 %autovettori
 vett1_x=V(1,1);
 vett1_y=V(2,1);
 autovettore1_x=[vett1_x 0];
 autovettore1_y=[vett1_y 0];
 m=U(2,1)/U(1,1);
 k=U(2,2)/U(1,2);
 b=linspace(-0.5,2);
 n=centro_y+m*(b-centro_x);
 c=centro_y+k*(b-centro_x);
 
 
 vett2_x=V(1,2);
 vett2_y=V(2,2);
 autovettore2_x=[vett2_x 0];
 autovettore2_y=[vett2_y 0];
 
 % EQUAZIONI BRACCI
          %braccio1
 braccio1_x=0;
 braccio1_y=0;
 braccio1_x1=a(1)*cos(teta(1));
 braccio1_y1=a(1)*sin(teta(1));
 braccio1x=[braccio1_x,braccio1_x1];
 braccio1y=[braccio1_y,braccio1_y1];
         %braccio2 
 braccio2_x1=a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
 braccio2_y1=a(1)*sin(teta(1))+a(2)*(sin(teta(1)+teta(2)));
 braccio2x=[braccio1_x1,braccio2_x1];
 braccio2y=[braccio1_y1,braccio2_y1];
 
 
%  q_punto= [J(1,1); J(2,1)];
%  v=J*q_punto;
%  v_x=v(1);
%  v_y=v(2);
%  Vxy=v'*(inv(J*J'))*v;
 
 % EQUAZIONE ELLISSE FORZA
 
 s1=G(1,1)*V(:,1)/norm(V(:,1)/norm(G));
 s2=G(2,2)*V(:,2)/norm(V(:,2)/norm(G));
 beta = linspace(0,2*pi);
 x = (s1(1))*cos(beta)+s2(1)*sin(beta);
 y = (s1(2))*cos(beta)+s2(2)*sin(beta);
          %centro
 x = x + centro_x;
 y = y + centro_y;
 
 
 %EQUAZIONE ELLISSE VELOCITA'
 %(D(1,1)*V(:,1)/norm(V(:,1))/norm(D),D(2,2)*V(:,2)/norm(V(:,2))/norm(D)
 
 
 s11=(D(1,1)*U(:,1)/norm(U(:,1)/norm(D)))/1000;
 s22=(D(2,2)*U(:,2)/norm(U(:,2)/norm(D)))/1000;
 x1 = s11(1)*cos(beta)+s22(1)*sin(beta);
 y1 = s11(2)*cos(beta)+s22(2)*sin(beta);
          %centro
 x1 = x1 + centro_x;
 y1 = y1 + centro_y; 
 plot(x1,y1,'-b','Linewidth',1.5,'Color','g')
 hold on
  plot(braccio1x,braccio1y,'.-k','markersize',15,'HandleVisibility','on')
  hold on
  plot(braccio2x,braccio2y,'.-k','markersize',15,'HandleVisibility','on')
  hold on 
   %%%%%%%%%%%%%%%%%%%%%%%%%%
  teta=[pi/12 -pi/9]
centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));

J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
     (a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (a(2)*(cos(teta(1)+teta(2))))];
 maniplobilitymesurement=sqrt(det((J)*(J')))
det(J)
%forza
 [V,G]=eig(J*J');
 %velocità
 [U,D]=eig((J*J')^-1);

 %autovettori
 vett1_x=V(1,1);
 vett1_y=V(2,1);
 autovettore1_x=[vett1_x 0];
 autovettore1_y=[vett1_y 0];
 m=U(2,1)/U(1,1);
 k=U(2,2)/U(1,2);
 b=linspace(-0.5,2);
 n=centro_y+m*(b-centro_x);
 c=centro_y+k*(b-centro_x);
 
 
 vett2_x=V(1,2);
 vett2_y=V(2,2);
 autovettore2_x=[vett2_x 0];
 autovettore2_y=[vett2_y 0];
 
 % EQUAZIONI BRACCI
          %braccio1
 braccio1_x=0;
 braccio1_y=0;
 braccio1_x1=a(1)*cos(teta(1));
 braccio1_y1=a(1)*sin(teta(1));
 braccio1x=[braccio1_x,braccio1_x1];
 braccio1y=[braccio1_y,braccio1_y1];
         %braccio2 
 braccio2_x1=a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
 braccio2_y1=a(1)*sin(teta(1))+a(2)*(sin(teta(1)+teta(2)));
 braccio2x=[braccio1_x1,braccio2_x1];
 braccio2y=[braccio1_y1,braccio2_y1];
 
 
 
 % EQUAZIONE ELLISSE FORZA
 
 s1=G(1,1)*V(:,1)/norm(V(:,1)/norm(G));
 s2=G(2,2)*V(:,2)/norm(V(:,2)/norm(G));
 beta = linspace(0,2*pi);
 x = (s1(1))*cos(beta)+s2(1)*sin(beta);
 y = (s1(2))*cos(beta)+s2(2)*sin(beta);
          %centro
 x = x + centro_x;
 y = y + centro_y;
 
 
 %EQUAZIONE ELLISSE VELOCITA'
 %(D(1,1)*V(:,1)/norm(V(:,1))/norm(D),D(2,2)*V(:,2)/norm(V(:,2))/norm(D)
 
 
 s11=(D(1,1)*U(:,1)/norm(U(:,1)/norm(D)))/1000;
 s22=(D(2,2)*U(:,2)/norm(U(:,2)/norm(D)))/1000;
 x1 = s11(1)*cos(beta)+s22(1)*sin(beta);
 y1 = s11(2)*cos(beta)+s22(2)*sin(beta);
          %centro
 x1 = x1 + centro_x;
 y1 = y1 + centro_y; 
 plot(x1,y1,'-b','Linewidth',0.2)

 hold on
  plot(braccio1x,braccio1y,'.-k','markersize',15,'HandleVisibility','on')
  hold on
  plot(braccio2x,braccio2y,'.-k','markersize',15,'HandleVisibility','on')
  legend("configurazione 1","configurazione 2","configurazione 3","configurazine 4 ")
  grid on
  axis('equal')
  set(gcf,'color','white')
  xlabel('[m]'),ylabel('[m]')
