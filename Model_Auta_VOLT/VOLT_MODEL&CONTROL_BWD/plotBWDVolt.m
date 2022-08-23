
%%%%%%%%%%%%%%%%%%%%%%%%% Plot function %%%%%%%%%%%%%%%%%%%%%%%%
xSize = 12; ySize = 7;
Xb = 300; Yb = 300;
xLeft = (21.5-xSize)/2; yTop = (27.9-ySize)/2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
set(gcf,'PaperUnits','centimeters');
set(gcf,'Units','centimeters');
set(gcf,'PaperPosition',[xLeft yTop xSize*50/37.81  ySize*50/37.81]);
set(gcf,'Position',[xLeft yTop xSize*50/37.81  ySize*50/37.81]);

subplot(211),plot(simtECMSBWD, mfECMSBWD,'b','LineWidth',2);
grid on;
xlabel('time[s]','FontSize',14,'FontName','Times');ylabel('m_f[g]','FontSize',14,'FontName','Times');
legend('m_f','location','NorthWest');

title(strcat('Fuel consumption m_f, m_f_,_t_o_t_a_l = ', mat2str(mfECMSBWD(end),6),' g'),'FontSize',14,'FontName','Times');
set(gca,'FontSize',14);
axis([0 simtECMSBWD(end) 0 max(mfECMSBWD)]);

subplot(212), plot(simtECMSBWD, soctECMSBWD, 'r','LineWidth',2);
if BLNDmode == 1
    hold on, plot(vrijeme, dekrement_SoC*vrijeme + Ssoc, 'k','LineWidth',2);
    legend('SoC', 'SoCR', 'location', 'southwest');
else
    legend('SoC','location','southwest');
end
grid on;
xlabel('time[s]','FontSize',14,'FontName','Times');ylabel('SoC','FontSize',14,'FontName','Times');

if BLNDmode == 1
    title(strcat(', BLND, Battery state of charge SoC, SoC = ',mat2str(100*soctECMSBWD(end),4),'%'),'FontSize',14,'FontName','Times');
else 
    title(strcat(', CD/CS, Battery state of charge SoC, SoC = ',mat2str(100*soctECMSBWD(end),4),'%'),'FontSize',14,'FontName','Times');
end

set(gca,'FontSize',14);
axis([0 simtECMSBWD(end) min(soctECMSBWD)-0.01 max(soctECMSBWD)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
set(gcf,'PaperUnits','centimeters');
set(gcf,'Units','centimeters');
set(gcf,'PaperPosition',[xLeft yTop xSize*50/37.81  ySize*50/37.81]);
set(gcf,'Position',[xLeft yTop xSize*50/37.81  ySize*50/37.81]);

subplot(311), plot(simtECMSBWD, weRECMSBWD,'b','LineWidth',2);
xlabel('time[s]','FontSize',14,'FontName','Times'), ylabel('\omega_e_R','FontSize',14,'FontName','Times');
%legend('BWD RB+ECMS','Location','NorthWest','orientation','horizontal');
grid on;
set(gca,'FontSize',14);
axis([0 simtECMSBWD(end) min(wmg1RECMSBWD) max(wmg1RECMSBWD)+10]);
    
subplot(312), plot(simtECMSBWD, teRECMSBWD,'r','LineWidth',2);
grid on; 
xlabel('time[s]','FontSize',14,'FontName','Times'), ylabel('\tau_e_R','FontSize',14,'FontName','Times');
%legend('BWD RB+ECMS','Location','NorthWest','orientation','horizontal');
set(gca,'FontSize',14);
axis([0 simtECMSBWD(end) min(teRECMSBWD) max(teRECMSBWD)+30]);
    
subplot(313),plot(simtECMSBWD, wmg1RECMSBWD.*teRECMSBWD/1000,'k','LineWidth',2)
grid on;
xlabel('time[s]','FontSize',14,'FontName','Times');
ylabel('P_e [kW]','FontSize',14,'FontName','Times');
%legend('BWD RB+ECMS','Location','NorthWest','orientation','horizontal');
set(gca,'FontSize',14);
axis([0 simtECMSBWD(end) min(wmg1RECMSBWD.*teRECMSBWD/1000) max(wmg1RECMSBWD.*teRECMSBWD/1000)+10]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
xSize = 12; ySize = 9;
set(gcf,'PaperUnits','centimeters');
set(gcf,'Units','centimeters');
set(gcf,'PaperPosition',[xLeft yTop xSize*50/37.81  ySize*50/37.81]);
set(gcf,'Position',[xLeft yTop xSize*50/37.81  ySize*50/37.81]);
load Aek.mat;

[X,Y] = meshgrid(1:525,1:141);
contour(X,Y,Aek,50);
colorbar;

hold on;
plot(tuice(:,1), tuice(:,2),'m','LineWidth',4);

x = 1:0.1:525;
plot(x, 0.0002*(x-105).^2 + granicaPar, 'b--', 'LineWidth', 3);
    
hold on, plot(weRECMSBWD, teRECMSBWD, 'kp', 'LineWidth', 1);

ylabel('\tau_e[Nm]','FontSize',14,'FontName','Times');
xlabel('\omega_e[rad/s]','FontSize',14,'FontName','Times');
set(gca,'FontSize',14);
  
if BLNDmode==1
    title(strcat(ciklus,', BLND, Engine optimal reference operating points'),'FontSize',14,'FontName','Times');
else 
    title(strcat(ciklus,', CD/CS, Engine optimal reference operating points'),'FontSize',14,'FontName','Times');
end
legend('\eta_I_C_E', '\tau_e_,_m_a_x', 'ICE op. region lower bound','Location', 'SouthEast');

axis([0 525 -0.1 140]);



