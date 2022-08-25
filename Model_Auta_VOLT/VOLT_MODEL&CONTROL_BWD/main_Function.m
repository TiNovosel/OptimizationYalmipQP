

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EREV (Chevrolet Volt) model + RB+ECMS upravljacka strategija %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all;

% Inicijalizacija parametara EREV pogona
initvolt;

%%%%%%%%%%%%%%% Odabir upravljacke strategije %%%%%%%%%%%%%%
chooseMode = 2;     % chooseMode = 1 --> samo EV nacin rada omogucen, chooseMode = 2 --> i EV i TMEV nacini rada su omoguceni
chooseControl = 0;  % Postavljanje upravljacke strategije: chooseControl == 0 --> RB+ECMS, chooseControl == 1 --> RB, chooseControl == 2 -->  RB+1D-ECMS
RBextended = 1;     % Dodatna pravila dobivena iz DP optimiranja su ukljucena u RB+ECMS upravljacku strategiju
BLNDmode = 0;       % Aktivacija kombiniranog nacina rada
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Postavljanje granica meðu nacinima rada + parametri
granicaPar = 48;    % Parametar koji odreduje donju granicu u mapi SUI motora unutar koje se radne tocke mogu postavljati
fix_hyst = 30;      % Parametar histereze granice izmedu hibridnih nacina rada
chooseBound = 2;    % Odabir razlicitih granica medu nacinima rada: chooseBound = 1 --> boundary + hysteresis, chooseBound = 2 --> GM boundary + hysteris, chooseBound = 3 --> boundary + GM boundary

Ssoc = 1;         % SoC na pocetku voznog ciklusa

% Postavljanje voznog ciklusa
ciklus = 'UDDS'; % NEDC, HWFET, UDDS, US
% Ponavljanje voznog ciklusa
broj_ponavljanja = 5; 

% 
plugged_and_charging = 0; % 0: vehicle on road and driving; 1: vehicle plugged and being charged
Pbatt_charging = -10000;     % Charging power (from grid)

% Ucitavanje voznog ciklusa i upravljaèkih parametera za širinu histereze
driving_cycle_load;
vrijeme = vrijeme;
% Simulacija modela EREV vozila
sim('voltECMSBWD', vrijeme(end));
plotBWDVolt
statistika
