clear all
close all
clc

%% Definiranje konstanti
nu = 4; %[Pc; Preg; Pdem; N_c] Pc - snaga punjenja, Preg - reg kocenje, Pdem - demanded power  , N_c - vektor priključenosti na mrežu
nx = 2; %[SoC ; C] 
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


VT = 2; %Viša tarifa - cijene struje
NT = 1; %Niža tarifa - cijene struje

N = (7*24*60)/dT; %Ukupni broj diskretnih trenutaka
N_k_in_day = (24*60)/dT;

x0 = []; %Početne vrijednosti

%% Definiranje dvotarifne cijene struje po diskretnom koraku
for r = 1:N_k_in_day
    if r < 32

        C_price_day(r,1) = NT;

    elseif r > 32 & r < 88 

        C_price_day(r,1) = VT;
    
    else

        C_price_day(r,1) = NT;
    end
end

for d = 1:(N/N_k_in_day)

    C_price = [C_price;C_price_day];

end



%% Vektor priključenosti na mrežu
% Priključenost na mrežu (Put pola sata do posla u 7 i povratak kući 45 min nakon 16h)
for n = 1:N_k_in_day
    if n <= 28   % 28 je 7 sati po 15 min

        n_c(n,1) = 1;

    elseif n <= 30 % 2x15min vožnje do posla

        n_c(n,1) = 0;

    elseif n <= 64

        n_c(n,1) = 1;

    elseif n <= 67

        n_c(n,1) = 0;
    else 

        n_c(n,1) = 1;
    end
end

% Priključenost na mrežu subotom(Putovanje 4 sata)
for j = 1:N_k_in_day

    if j >= 28 & j <= 44 

        n_c_s(j,1) = 0;

    else 
        n_c_s(j,1) = 1;
    end

end

% Priključenost na mrežu nedjeljom (Povratak s putovanja 4 sata)
for j = 1:N_k_in_day

    if j >= 64 & j <= 80 

        n_c_n(j,1) = 0;

    else 
        n_c_n(j,1) = 1;
    end

end

% Ukupni vektor priključenosti na mrežu za cijeli tjedan
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

P_dem = []; %dobiti iz modela, ali sad su ovak random samo za isprobati

for i = 1:N
    if N_c(i,1) == 1
        P_dem(i,1) = 0;
    else
        P_dem(i,1) = 0.2; 
    end
end


P_reg = zeros(N,1); %dobiti iz modela






%% Yalmip optimizacija

% u = [Pc; Preg; Pdem; N_c]
% x = [SoC ; C]

yalmip('clear')

x0 = [SoC0; C_price(1,1)]; % početne vrijednosti 

u = sdpvar(nu, N);
x = sdpvar(nx, N+1);

constraints = [];
objective = 0;
for k = 1:N
 objective = objective + (x(2,k)*u(1,k)*(dT/1000))*Alfa + (u(1,k)*u(1,k))*(1-Alfa)
 constraints = [constraints, x(1, k+1) == x(1, k) + ndch*(u(1,k)+u(2,k))*dT/Emax -u(3,k)*(dT/(nch*Emax))]; 
 constraints = [constraints, x(2, k) == C_price(k), u(2,k) == P_reg(k), u(3,k) == P_dem(k)];
 constraints = [constraints, 0.3 <= x(1,k)<= 1, 0 <= u(1,k)<= 10000];
 constraints = [constraints, u(1,k) == u(1,k)*u(4,k)];
end

Optimal_Pc = optimizer(constraints, objective,[],x(:,1),u(:,:)) %objekt

Optimal_Pc_Solution = Optimal_Pc(x0);

