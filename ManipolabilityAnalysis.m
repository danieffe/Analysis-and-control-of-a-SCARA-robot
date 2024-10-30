clc
clear all
close all

a=[0.5 0.5];

%teta=[pi/4 -pi/2]; % Ellissoide 1)
%teta=[pi/3 -pi/3]; % Ellissoide 2)
%teta=[pi/6 -pi/4]; % Elissoide 3)
%teta=[pi/6 pi/4]; % Elissoide 4)

teta=[pi/4 -pi/2] %questo è a gomito basso


centro_x= a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)));
centro_y= a(1)*sin(teta(1))+ a(2)*sin(teta(1)+teta(2));

J= [-a(1)*sin(teta(1))-a(2)*(sin((teta(1)+teta(2)))) , (-a(2)*(sin(teta(1)+teta(2))));
     (a(1)*cos(teta(1))+a(2)*(cos(teta(1)+teta(2)))) , (a(2)*(cos(teta(1)+teta(2))))];
 
 
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
 
      
       %plot
figure()
plot(x,y,'-r','Linewidth',1.5)
 hold on
plot(x1,y1,'-b','Linewidth',1.5)
legend('$Ellisse in velocita$','$Ellisse in forza$','interpreter','latex','FontSize',11)
xlabel('$[m]$','interpreter','latex','FontSize',10)
ylabel('$[m]$','interpreter','latex','FontSize',10')
axis('equal')
% grid on
% plot(x,y,'b')
% hold on
% plot(x1,y1,'r')
   hold on
  plot(braccio1x,braccio1y,'.-k','markersize',15)
  hold on
  plot(braccio2x,braccio2y,'.-k','markersize',15)
  grid on
% %   plot(autovettore1_x,autovettore1_y,'r')
% %  hold on
% %   plot(autovettore2_x,autovettore2_y,'r')
%   hold on
%   plot(b,n)
%   hold on
%   plot(b,c)
% %  
%  %plot(Vx,y)
% %  hold on
% %  plot(x0,y0,'k')
% %  hold on
  % plot(x1,y1,'k')
% % axis([-2 3 -1 2])
  axis('equal')
%  title('Analisi di manipolabilità')
%  legend('Ellisse in forza','Ellisse in velocità')
%  grid on
%  xlabel('[m]')
%  ylabel('[m]')
%  
%  
% 
%  