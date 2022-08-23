


if strcmp(ciklus, 'NEDC')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cpoints = [];
    cpoints = load(strcat(ciklus,'t-tuL-wL','.txt')); 
    %% Power standard deviation and hysteresis width calculation parameters set %%
    bufferSize = 26; % Buffer width expressed in seconds
    hystD = 30;      % lower limit of hysteresis width
    hystG = 70;      % upper limit of hysteresis width
    tcdstdD = 0;     % lower torque limit [Nm]    
    tcdstdG = 300;   % upper torque limit [Nm]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(ciklus, 'HWFET')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cpoints = [];
    cpoints = load(strcat(ciklus,'t-tuL-wL','.txt')); 
    %% Power standard deviation and hysteresis width calcualtion parameters set %%
    bufferSize = 10; % Buffer width expressed in seconds
    hystD = 30;      % lower limit of hysteresis width
    hystG = 70;      % upper limit of hysteresis width
    tcdstdD = 0;     % lower torque limit [Nm]    
    tcdstdG = 750;   % upper torque limit [Nm]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(ciklus, 'UDDS')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cpoints = [];
    cpoints = load(strcat(ciklus,'t-tuL-wL','.txt')); 
    %% Power standard deviation and hysteresis width calcualtion parameters set %%
    bufferSize = 54; % Buffer width expressed in seconds
    hystD = 30;      % lower limit of hysteresis width
    hystG = 70;      % upper limit of hysteresis width
    tcdstdD = 0;     % lower torque limit [Nm]    
    tcdstdG = 300;   % upper torque limit [Nm]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(ciklus, 'US')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cpoints = [];
    cpoints = load(strcat(ciklus,'t-tuL-wL','.txt')); 
    %% Power standard deviation and hysteresis width calcualtion parameters set %%
    bufferSize = 28; % Buffer width expressed in seconds
    hystD = 30;      % lower limit of hysteresis width
    hystG = 70;      % upper limit of hysteresis width
    tcdstdD = 0;     % lower torque limit [Nm]    
    tcdstdG = 650;   % upper torque limit [Nm]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif strcmp(ciklus,'SVI')
    cpoints1 = load('NEDCt-tuL-wL.txt'); 
    cpoints2 = load('HWFETt-tuL-wL.txt'); 
    cpoints3 = load('UDDSt-tuL-wL.txt'); 
    cpoints4 = load('USt-tuL-wL.txt'); 
    cpoints = [cpoints1;cpoints2;cpoints3;cpoints4];
end
 
prirodni_ciklusi = '0';

if strcmp(prirodni_ciklusi,'0')   
    % Ucitavanje ciklusa
    pom = [];
    for i = 1:broj_ponavljanja
       pom = [pom; cpoints]; 
    end
    cpoints = pom;
    vrijeme = 1:size(cpoints,1);
    cbrzina = cpoints(:,3);    
    cmoment = cpoints(:,2);
    dekrement_SoC = 1*(0.3-0.95)/size(cpoints,1);
end