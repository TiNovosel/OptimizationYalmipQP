clear all
close all
clc

load simulacija_podaci.mat Pbat  %vrijedosti iz simulacije 

%% Definiranje konstanti
nu = 1; %Pc - snaga punjenja
nx = 1; %SoC
dT = 15; %min


nch = 0.9; % Korisnost punjenja
ndch = 0.9; % Korisnost pražnjenja

SoC0 = 1; %Vrijednost SoC-a u početnom trenutku - 1 = 100%

Emax = 16000; %Wh pogledati jos poslje

C_price_day = []; % Dnevni vektor cijena po diksretnom koraku
C_price = []; % Tjedni vektor cijene po diksretnom koraku

N_c = []; % Ukupni pokazatelj priključenosti na mrežu
n_c = []; % Pokazatelj priključenosti na mrežu
n_c_s = []; % Priljučenost na mrežu subota
n_c_n = []; % Priključenost na mrežu nedjeljom


VT = 0.9535*1/4; %Viša tarifa - cijene struje
NT = 0.4456*1/4; %Niža tarifa - cijene struje

N = (7*24*60)/dT; %Ukupni broj diskretnih trenutaka
N_k_in_day = (24*60)/dT;


a%% Definiranje dvotarifne cijene struje po diskretnom koraku
for r = 1:N_k_in_day
    if r < 32

        C_price_day(r,1) = NT;

    elseif r > 32 & r < 88

        C_price_day(r,1) = VT;
    
    else

        C_price_day(r,1) = NT;
    end
end


% Definiranje tjednog vektora za dvotarifnu cijenu -->C_price
for d = 1:(N/N_k_in_day)

    C_price = [C_price;C_price_day]; %Dvotarfini tijedni vektor

end



%% Vektor priključenosti na mrežu
% Priključenost na mrežu (Put 30min do posla u 7 i povratak kući 30min nakon 16h)-> n_c 
for n = 1:N_k_in_day
    if n <= 28   % 28 je 7 sati po 15 min

        n_c(n,1) = 1;

    elseif n <= 30 % 2x15min vožnje do posla

        n_c(n,1) = 0;

    elseif n <= 64

        n_c(n,1) = 1;

    elseif n <= 66

        n_c(n,1) = 0;
    else 

        n_c(n,1) = 1;
    end
end

% Priključenost na mrežu subotom(Putovanje 4 sata)-> n_c_s
for j = 1:N_k_in_day

    if j > 40 && j <= 52 

        n_c_s(j,1) = 0;

    else 
        n_c_s(j,1) = 1;
    end

end

% Priključenost na mrežu nedjeljom (Povratak s putovanja 4 sata)-> n_c_n
for j = 1:N_k_in_day

    if j > 56 && j <= 68 

        n_c_n(j,1) = 0;

    else 
        n_c_n(j,1) = 1;
    end

end

% Ukupni vektor priključenosti na mrežu za cijeli tjedan --> N_c 
for c = 1:(N/N_k_in_day)

    if c <= 5 

        N_c = [N_c; n_c];

    elseif c == 6

        N_c = [N_c; n_c_s];

    else
        N_c = [N_c; n_c_n];

    end

end


%% Snage 

% Simulacijski vektor Pbat dolazi na sekundnoj bazi s pozitivnim i
% negativnim snagama sto razdvajamo u dva vektora jer je tako ulaz u
% dinamiku baterije.

%Razdvojeno u dva vektora 
P_dem_s = []; 
P_reg_s = []; 

for kk = 1:size(Pbat) % -> P_dem_s, P_reg_s 
    if Pbat(kk,1) >= 0
        P_dem_s(kk,1) = Pbat(kk,1);
        P_reg_s(kk,1) = 0;
    else
        P_reg_s(kk,1) = Pbat(kk,1);
        P_dem_s(kk,1) = 0;
    end
end

% Kracenje vektora na 5400 sekundi --> 6x 15 min
P_dem_s = P_dem_s(1:5400,1);
P_reg_s = P_reg_s(1:5400,1);



% Normalizacija na 900 sec -> 15 min 
P_dem_n = []; 
P_reg_n = []; 
second_counter_15min = 1;
sum_sec = 0;
minutes_count = 0;

for cc = 1:size(P_dem_s) % -> P_dem_n
    if second_counter_15min < 900
        sum_sec = sum_sec + P_dem_s(cc,1);
        second_counter_15min = second_counter_15min + 1;
    else
        minutes_count = minutes_count + 1;
        P_dem_n(minutes_count,1) = sum_sec;
        second_counter_15min = 1;
        sum_sec = 0;
    end
end


sum_sec = 0;
minutes_count = 0;

for cc = 1:size(P_reg_s) % -> P_reg_n
    if second_counter_15min < 900
        sum_sec = sum_sec + P_reg_s(cc,1);
        second_counter_15min = second_counter_15min + 1;
    else
        minutes_count = minutes_count + 1;
        P_reg_n(minutes_count,1) = sum_sec;
        second_counter_15min = 1;
        sum_sec = 0;
    end
end


% Određivanje vektora trošenja(dem) i dobivene snage regenom(reg) za cijeli
% dan na delta t = 15min
Pdem_radni_dan = [];
ccc = 1;

for Pdrd = 1 : n % -> Pdem_radni_dan
    if Pdrd <= 28   % 28 je 7 sati po 15 min

        Pdem_radni_dan(Pdrd,1) = 0;

    elseif Pdrd <= 30 % 2x15min vožnje do posla
        Pdem_radni_dan(Pdrd,1) = P_dem_n(ccc,1);
        ccc = ccc + 1; 

    elseif Pdrd <= 64

        Pdem_radni_dan(Pdrd,1) = 0;
        ccc = 3;

    elseif Pdrd <= 66
        Pdem_radni_dan(Pdrd,1) = P_dem_n(ccc,1);
        ccc = ccc + 1; 
    else 

        Pdem_radni_dan(Pdrd,1) = 0;
    end
end



Preg_radni_dan = [];
kkk = 1;

for Prrd = 1 : n % -> Preg_radni_dan
    if Prrd <= 28   % 28 je 7 sati po 15 min

        Preg_radni_dan(Prrd,1) = 0;

    elseif Prrd <= 30 % 2x15min vožnje do posla
        Preg_radni_dan(Prrd,1) = P_reg_n(kkk,1);
        kkk = kkk + 1; 

    elseif Prrd <= 64

        Preg_radni_dan(Prrd,1) = 0;
        kkk = 3;

    elseif Prrd <= 66
        Preg_radni_dan(Prrd,1) = P_reg_n(kkk,1);
        kkk = kkk + 1; 
    else 

        Preg_radni_dan(Prrd,1) = 0;
    end
end



Pdem_sub = [];
fff = 1;

% Snaga potrošena subotom(Putovanje 3 sata)
for Pds = 1:n % -> Pdem_sub

    if Pds > 40 && Pds <= 52 

        Pdem_sub(Pds,1) = P_dem_n(fff,1);
        fff = fff + 1;
        if fff > 6
            fff = 1;
        end
    else 
        Pdem_sub(Pds,1) = 0;
    end

end


% Snaga potrošena nedjeljom (Povratak s putovanja 3 sata)
Pdem_ned = [];
fff = 1;

for Pdn = 1:n % -> Pdem_ned

    if Pdn > 56 && Pdn <= 68 

        Pdem_ned(Pdn,1) = P_dem_n(fff,1);
        fff = fff + 1;
        if fff > 6
            fff = 1;
        end

    else 
        Pdem_ned(Pdn,1) = 0;
    end

end


Preg_sub = [];
ddd = 1;

% Snaga dobivena kocenjem subotom(Putovanje 3 sata)
for Prs = 1:n % -> Preg_sub

    if Prs > 40 && Prs <= 52 

        Preg_sub(Prs,1) = P_reg_n(ddd,1);
        ddd = ddd + 1;
        if ddd > 6
            ddd = 1;
        end
    else 
        Preg_sub(Prs,1) = 0;
    end

end


% Snaga dobivena kocenjem nedjeljom (Povratak s putovanja 4 sata)
Preg_ned = [];
ddd = 1;

for Prn = 1:n % -> Preg_ned

    if Prn > 56 && Prn <= 68 

        Preg_ned(Prn,1) = P_reg_n(ddd,1);
        ddd = ddd + 1;
        if ddd > 6
            ddd = 1;
        end

    else 
        Preg_ned(Prn,1) = 0;
    end

end



%Vektori vrijednosti trazenog i regen vracene snage za cijeli horizont
Pdem = [];   
Preg = [];

% Ukupni vektor trazene snage za cijeli tjedan
for pmin = 1:(N/N_k_in_day) % -> Pdem

    if pmin <= 5 

        Pdem = [Pdem; Pdem_radni_dan];

    elseif c == 6

        Pdem = [Pdem; Pdem_sub];

    else
        Pdem = [Pdem; Pdem_ned];

    end

end

% Ukupni vektor regen snage za cijeli tjedan
for ppmin = 1:(N/N_k_in_day) % -> Preg

    if ppmin <= 5 

        Preg = [Preg; Preg_radni_dan];

    elseif c == 6

        Preg = [Preg; Preg_sub];

    else
        Preg = [Preg; Preg_ned];

    end

end


%% Yalmip optimizacija
Alfa = 0.5; % Tezinski koeficjent

yalmip('clear')


x0 = [SoC0]; % početne vrijednosti 

u = sdpvar(nu, N);  %P_c
x = sdpvar(nx, N+1); %SoC

constraints = [];
objective = 0;

for k = 1:N
    objective = objective + (C_price(k,1)*u(1,k)*(dT/1000))*Alfa + (u(1,k)*u(1,k))*(1-Alfa);
    if Pdem(k,1) > 0
        constraints = [constraints, u(1,k) == 0];
    end
    constraints = [constraints, x(1,k+1) == x(1,k) + ndch*(u(1,k)+Preg(k,1))*dT/Emax -Pdem(k,1)*(dT/(nch*Emax))];
    constraints = [constraints, 0 <= x(1,k)<= 1, 0 <= u(1,k)<= 10000];
    if k == N
        constraints = [constraints, x(1,k+1) == 1];
    end
end

Optimal_Pc = optimizer(constraints, objective,[],x(:,1),u(:,:)) %objekt

Optimal_Pc_Solution = Optimal_Pc(x0)




% %% Plot
% %Plot snage dobivene iz simulacije:
% Pbat = Pbat(1:5400,1);
% %plot(Pbat, 'b--', 'LineWidth', 3);
% title(strcat('Snaga pražnjenja i punjenja za vrijeme vožnje'),'FontSize',14,'FontName','Times');
% hold on, plot(P_dem_s,'r','LineWidth', 1);
% hold on, plot(P_reg_s,'b','LineWidth', 1);
% ylabel('P[W]','FontSize',14,'FontName','Times');
% xlabel('t[sec]','FontSize',14,'FontName','Times');
% legend("P_{dem}","P_{reg}")
% set(gca,'FontSize',14);
% xlim([0 5400]);
% 
% %% Plot
% %Plot snage dobivene iz simulacije:
% %plot(Pbat, 'b--', 'LineWidth', 3);
% title(strcat('Tjedni profil snage po diskretnom vremenu'),'FontSize',14,'FontName','Times');
% hold on, plot(Pdem,'r','LineWidth', 1);
% hold on, plot(Preg,'b','LineWidth', 1);
% ylabel('P[W]','FontSize',14,'FontName','Times');
% xlabel('\Delta T','FontSize',14,'FontName','Times');
% legend("P_{dem}","P_{reg}")
% set(gca,'FontSize',14);
% xlim([0 672]);