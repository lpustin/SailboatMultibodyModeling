clear; clc; close all;

dati_input



f=@(x) -calc_V_boat(x)


options = optimset('PlotFcns',@optimplotfval);
options.MaxIter=20;
options.TolX=0.1;
x0=3
x = fminsearch(f,x0,options)






function V_boat = calc_V_boat(teta_MS)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    assignin('base','teta_MS',teta_MS)
    %%%% 'SaveState','on','StateSaveName','xout',...
    
    simOut = sim('CLASSEA_ok','SimulationMode','normal','SaveOutput','on','OutputSaveName','yout');
	V_boat=simOut.yout{1}.Values.Data(end);
    
end
