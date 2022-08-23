
% clc
gal2lit = 3.7854118;
m2km = 1.6093;
% Postotak aktivnosti 1D-ECMS-a
ECMS1D_percentage = sum(abs(fjaW))/size(fjaW,1)
% Postotak aktivnosti 2D-ECMS-a
ECMS2D_percentage = sum(1-abs(fjaW))/size(fjaW,1)
% Prosjecan Aek [g/kWh]
Aek_mean = mean(mf_stat(Pe_stat>10)./Pe_stat(Pe_stat>10)*3.6e6)
Aek_min = min(mf_stat(Pe_stat>10)./Pe_stat(Pe_stat>10)*3.6e6)
% Potrosnja  s
dist_km = dist_travel(end)/1000 
grams_consumption = mfECMSBWD(end)
liters_consumption = mfECMSBWD(end)/820
gallon_consumption = liters_consumption/gal2lit
liters_per_100km = 100*liters_consumption/dist_km
miles_per_gallon = dist_km/m2km/gallon_consumption