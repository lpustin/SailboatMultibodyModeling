clear; clc; close all;


%% simulation parameters 

Tmax=15;

%ode45 pars
MaxStepSize=1e-2;
RelTol=1e-2;
AbsTol=1e-2;
% RelTol=1e-1;
% AbsTol=1e-1;

Fil_t_const=5e-2; %filters to compute signals derivatives for symscape




%% wind speed and boat initial speed
V_wind=8; %m/s
V_boat_in=3.8; %m/s

% V_wind=4; %m/s
% V_boat_in=2.0; %m/s





%% initial conditions
COG_tg=45; %[deg]

SCARROCCIO_in=2; %[deg]
TWA=COG_tg-SCARROCCIO_in; %[deg]

surge_in=0;
sway_in=0;
heave_in=0;
roll_in=deg2rad(5);
pitch_in=deg2rad(0);
yaw_in=deg2rad(TWA);

surgep_in=-V_boat_in*cos(COG_tg*pi/180);
swayp_in=-V_boat_in*sin(COG_tg*pi/180);


teta_MS_in=atan2(swayp_in,surgep_in-V_wind); %[deg] try to initialize with zero AoA




%% inerzia
m_crew=80; %kg


m_mast=10; %kg

m_hull=30; %kg



%% geometric param

x_off_mast=0.5; %%CONTROLLARE

H_mast=9;

L_boom=2;


L_hull=5.49; %m
Width=2.50; %m VERIFICARE
H_hull=0.40; %m

l_beam=2.3; %m


%% control
% teta_MS = 9.7; % [deg] NOT USED 
% RD_angle=0; %deg NOT USED 

max_crew_y=Width+0.8;

AoA_MS_opt=15; %[deg] the controller try to keep AoA_MS_opt until roll<10deg then it reduce the AoA_MS target


%% aero and hydro param

gravity=-9.81; %m/s2
rho_h=1025; %kg/m3  sea water


%mainsail
param_MS.AeroCentr=[0,0,2];
% param_MS.AeroCentr=[0,0,0];
param_MS.isBiphase=false;

param_MS.rho=1.25; %[kg/m3] air
param_MS.V_wind=V_wind;
param_MS.S=14; %m2
param_MS.span=H_mast; %m
param_MS.cl_alpha=5; 
param_MS.cm_alpha=0.5; 
param_MS.cd0=0.01; 
param_MS.AR=H_mast^2/param_MS.S; 
param_MS.e=0.8;



% centerboard
c_CB=0.25; %m chord
h_CB=1.5; %m
Mass_CB=3; %kg
x_CB=0.2; %m CB offset MEAN HULL origin
TILT_CB=0; %deg
% TILT_CB=45; %deg  A STARTING POINT FOR FOIL MODELING

param_CB.AeroCentr=[0,0,-h_CB/2];
% param_CB.AeroCentr=[0,0,0];

param_CB.isBiphase=true;
param_CB.rho=1025;
param_CB.V_wind=0;
param_CB.S=c_CB*h_CB; %m2
param_CB.span=h_CB; %m
param_CB.cl_alpha=5; 
param_CB.cm_alpha=0.5; 
param_CB.cd0=0.01; 
param_CB.AR=h_CB^2/param_CB.S; 
param_CB.e=0.8;



% RUDDER
c_RD=0.2; %m chord
h_RD=1; %m
Mass_RD=2; %kg
x_RD=L_hull/2; %m CB offset MEAN HULL origin

param_RD.AeroCentr=[0,0,-h_RD/2];
% param_RD.AeroCentr=[0,0,0];
param_RD.isBiphase=true;

param_RD.rho=1025;
param_RD.V_wind=0;
param_RD.S=c_RD*h_RD; %m2
param_RD.span=h_RD; %m
param_RD.cl_alpha=5; 
param_RD.cm_alpha=0.5; 
param_RD.cd0=0.01; 
param_RD.AR=h_RD^2/param_RD.S; 
param_RD.e=0.8;



% T-RUDDER FOIL
param_T_RD.AeroCentr=[0,0,0];
param_T_RD.isBiphase=false;

c_T_RD=0.1; %m chord
h_T_RD=0.2; %m
Mass_T_RD=0.2; %kg

param_T_RD.rho=1025;
param_T_RD.V_wind=0;
% param_T_RD.S=c_T_RD*h_T_RD; %m2 A STARTING POINT FOR FOIL MODELING
param_T_RD.S=0; %m2 %% THE CONTRIBUTION IS NULL!!
param_T_RD.span=h_T_RD; %m
param_T_RD.cl_alpha=5; 
param_T_RD.cm_alpha=0.5; 
param_T_RD.cd0=0.01; 
param_T_RD.AR=h_T_RD^2/param_T_RD.S; 
param_T_RD.e=0.8;







%% rot matrix
y_to_z=[1  0   0
        0  0  -1
        0  1   0];
    
z_to_y = inv(y_to_z);


x_to_z=[0  0  -1
        0  1   0
        1  0   0];
    
z_to_x = inv(x_to_z);