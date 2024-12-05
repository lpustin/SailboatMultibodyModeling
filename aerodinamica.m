function aerodinamica(block)
% Level-2 MATLAB file S-Function for continuous time variable step demo.
%   Copyright 1990-2009 The MathWorks, Inc.
%   $Revision: 1.1.6.2 $

setup(block);

function setup(block)

%% Register number of dialog parameters
block.NumDialogPrms = 1;
%% Register number of input and output ports

block.NumInputPorts  = 8; % Rot_Matr, w, wp, Xo_SR_Bod, Vo, lamda_0
block.NumOutputPorts = 5; % Forces, Xo, V_wind_tip
%% Setup functional port properties to dynamically inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

block.InputPort(1).Dimensions        = 4; % quatern
block.InputPort(1).DirectFeedthrough = false;

block.InputPort(2).Dimensions        = 3; %  w
block.InputPort(2).DirectFeedthrough = false;

block.InputPort(3).Dimensions        = 3; %  wp
block.InputPort(3).DirectFeedthrough = false;

block.InputPort(4).Dimensions        = 3; % Xo_SR_Bod
block.InputPort(4).DirectFeedthrough = false;

block.InputPort(5).Dimensions        = 3; %  Vo
block.InputPort(5).DirectFeedthrough = false;

block.InputPort(6).Dimensions        = 1; %  lamda_0
block.InputPort(6).DirectFeedthrough = false;

block.InputPort(7).Dimensions        = 4; % quatern SR PIL
block.InputPort(7).DirectFeedthrough = false;

block.InputPort(8).Dimensions        = 3; % x_0_sr_pil
block.InputPort(8).DirectFeedthrough = false;



block.OutputPort(1).Dimensions       = 6; % Forces
block.OutputPort(2).Dimensions       = 3; % Xo
block.OutputPort(3).Dimensions       = 1; % V_wind_tip
block.OutputPort(4).Dimensions       = 31; % AoAs
block.OutputPort(5).Dimensions       = 31; % Cls





%% Set the block simStateCompliance to default (i.e., same as a built-in block)

block.SimStateCompliance = 'DefaultSimState';

%% Set up the continuous states.

block.NumContStates = 0;

%% Set block sample time to continuous

block.InputPort(1).SampleTime =[0 0];
block.InputPort(2).SampleTime =[0 0];
block.InputPort(3).SampleTime =[0 0];
block.InputPort(4).SampleTime =[0 0];
block.InputPort(5).SampleTime =[0 0];
block.InputPort(6).SampleTime =[0 0];
block.InputPort(7).SampleTime =[0 0];
block.InputPort(8).SampleTime =[0 0];


block.InputPort(1).SamplingMode = 'Sample';
block.InputPort(2).SamplingMode = 'Sample';
block.InputPort(3).SamplingMode = 'Sample';
block.InputPort(4).SamplingMode = 'Sample';
block.InputPort(5).SamplingMode = 'Sample';
block.InputPort(6).SamplingMode = 'Sample';
block.InputPort(7).SamplingMode = 'Sample';
block.InputPort(8).SamplingMode = 'Sample';



block.OutputPort(1).SampleTime = [0 0];
block.OutputPort(2).SampleTime = [0 0];
block.OutputPort(3).SampleTime = [0 0];
block.OutputPort(4).SampleTime = [0 0];
block.OutputPort(5).SampleTime = [0 0];


block.OutputPort(1).SamplingMode = 'Sample';
block.OutputPort(2).SamplingMode = 'Sample';
block.OutputPort(3).SamplingMode = 'Sample';
block.OutputPort(4).SamplingMode = 'Sample';
block.OutputPort(5).SamplingMode = 'Sample';




%% Register methods
block.RegBlockMethod('Outputs',                 @Output);
block.RegBlockMethod('SetInputPortSampleTime', @SetInputPortSampleTime);
block.RegBlockMethod('SetOutputPortSampleTime', @SetOutputPortSampleTime);

function Output(block)
%     tic

    %parametri
    in=block.DialogPrm(1).Data; %struttura par_inflow

    %Ingressi
    quatern=block.InputPort(1).Data;
    w=block.InputPort(2).Data;
    wp=block.InputPort(3).Data;
    Xo_SR_Bod=block.InputPort(4).Data;
    Vo=block.InputPort(5).Data;
    lamda_0=block.InputPort(6).Data;
    quat_pil=block.InputPort(7).Data;

    x_0_SR_pil=block.InputPort(8).Data;

    
    %controllo sui quaternioni
    problema_iniziale=[0;0;0;0];
    
    if isequal(quatern, problema_iniziale)
        disp('PROBLEMA INIZIALE QUATERN 0')
        quatern=[1;0;0;0];
    end
    
    if isequal(quatern, problema_iniziale)
        disp('PROBLEMA INIZIALE QUATERN 0')
        quat_pil=[1;0;0;0];
    end
    

    
    Rot_Matr=quat2rotm(quatern'); %dal SR pala corpo al SR terra
    
    Rot_Matr_pil=quat2rotm(quat_pil'); %dal SR PILONE CORPO al SR terra
    
    x_0_SR_pil=Rot_Matr_pil*x_0_SR_pil; %porto la x_0_SR_pil nel SR terra
    
%     disp("quat_pil")
%     disp( quat_pil )
    
    Rot_Matr_pil=inv(Rot_Matr_pil); %dal SR Terra al SR PILONE corpo
    %porto la X3_4 nel SR pilone corpo per prendere le giuste x e y da
    %mettere nella soluzione potenziale 2d del tower shadow
    
    %NB VEDERE COME FARE SE AGGIUNGO SURGE SWAY...
%     
%     disp("Rot_Matr")
%     disp(Rot_Matr)
%     disp("w")
%     disp(w)
%     disp("Xo_SR_Bod")
%     disp(Xo_SR_Bod)
%     disp("Vo")
%     disp(Vo)
%     disp("lamda_0")
%     disp(lamda_0)

    
    Xo=Rot_Matr*Xo_SR_Bod; %porto la Xo (espressa nel SR corpo) nel SR terra
    
    
%     f_deb=fopen('debug_pala', 'w');
% % %     fprintf(f_deb, "x[m]\tVx[m/s]\tVy[m/s]\tVz[m/s]\tbeta[deg]\tAoA[deg]\tcl\tcd0\tcdi\tcm\tdL\tdD\tdFy\tdFz\tdMx\tdMy\tdMz\n");
%     fprintf(f_deb, "x[m]         Vx[m/s]      Vy[m/s]    Vz[m/s]    beta[deg]   AoA[deg]      cl         cd0          cdi         cm          dL            dD         dFy          dFz        dMx        dMy          dMz       x_o       y_o      z_o\n");


%shared variables
    dF=zeros(3,numel(in.x));
    dM=zeros(3,numel(in.x));
    AoAs=zeros(1,numel(in.x));
    Cls=zeros(1,numel(in.x));


    for i=1:numel(in.x)
% private variables      

        %calcolo la posizione di X_3_4 nel SR corpo e terra
        R_3_4=[in.x(i); -0.25*in.c(i); 0];
        X_3_4=Xo+Rot_Matr*R_3_4; % aggiungo alla Xo la R_3_4 nel SR terra
        
        %NB AGGIUNGERE IL CONTRIBUTO X_0_SR_PIL NEL CASO DI SURGE SWAY
        %la x_3_4 nel SR pilone questa va usata per il resto
        X_3_4_SR_pil=Rot_Matr_pil*(X_3_4-x_0_SR_pil);
        
        %effetto tower shadow e wind shear
        %NB la z per il calcolo del wind shear presa dalla X3_4 nel SR terra
        V_h = in.U_r_sh * log( X_3_4_SR_pil(3) /in.z_0_sh)/log(in.z_r_sh/in.z_0_sh);
        R_pil = in.R_pil_dw + (in.R_pil_up-in.R_pil_dw) * (X_3_4_SR_pil(3)/in.h_pil);
        
        if X_3_4_SR_pil(3) > in.h_pil
            R_pil=0;
        end
        
        
        V_tot_gr = V_h +  V_h*( R_pil^2*(X_3_4_SR_pil(2)^2-X_3_4_SR_pil(1)^2) ) / (X_3_4_SR_pil(1)^2+X_3_4_SR_pil(2)^2)^2;

        V_tot_gr=V_tot_gr-lamda_0;

        %%%%% NB se voglio const wind
%         V_tot_gr=in.U_r_sh-lamda_0;
        
        
        V_w=Rot_Matr'*[-V_tot_gr; 0; 0]; %porto la V_wind nel SR corpo
        V = Vo + cross(w,R_3_4) + V_w; %velocità 3_4 corda
        
        
        beta = atan2( -V(3), -V(2) ); %angolo tra il vento e SR corpo al root
        AoA = beta-in.sverg(i); % AoA ottenuto come beta-svergolamento
        

        
        if isnan(beta)
            disp('PROBLEMA BETA NANIII')
        end
        if AoA>pi | AoA<-pi
            AoA=mod(AoA,pi);
            disp('PROBLEMA AOA!!!')
            disp(AoA*180/pi)
        end
%         if AoA<-pi
%             AoA=mod(AoA,-pi);
%             disp('PROBLEMA AOA!!!')
%             disp(AoA*180/pi)
%         end
        
        AoA_interp=1800+round(AoA*180/pi*10)+1;
        %NBBBB *10 perchè HO CAMBIATO LO STEP A 0.1 GRADI DI ALPHA
        
    
%         Ut= V(2)*cos(in.sverg(i)) + V(3)*sin(in.sverg(i));
%         Un=-V(2)*sin(in.sverg(i)) + V(3)*cos(in.sverg(i));
%         Ut=-Ut;
%         Un=-Un;
%         alpha=atan2(Un,Ut); %controllato!!       


        %interpolazione coefficienti
        %indice x corrisponde a i ho già interpolato sulle x aerodinamiche
        %indice di AoA dato che alpha ho usato passo 1 è:
        
% 
%         cl =2*pi*AoA;
%         cd0 =0.01;
%         cm =0.0;
%         
        cl =in.cl(AoA_interp-1, i) + ( AoA-in.alpha(AoA_interp-1) )*( in.cl(AoA_interp,i)-in.cl(AoA_interp-1, i) )/ ( in.alpha(AoA_interp)-in.alpha(AoA_interp-1) );
        cd0 =in.cd0(AoA_interp-1, i) + ( AoA-in.alpha(AoA_interp-1) )*( in.cd0(AoA_interp,i)-in.cd0(AoA_interp-1, i) )/ ( in.alpha(AoA_interp)-in.alpha(AoA_interp-1) );
        cm =in.cm(AoA_interp-1, i) + ( AoA-in.alpha(AoA_interp-1) )*( in.cm(AoA_interp,i)-in.cm(AoA_interp-1, i) )/ ( in.alpha(AoA_interp)-in.alpha(AoA_interp-1) );
        
  
%         display([in.x(i), AoA*180/pi, cl] )

        dL = 0.5*in.rho*in.c(i)*norm(V)^2*cl;
        dD = 0.5*in.rho*in.c(i)*norm(V)^2*cd0;
     
        dF_private=[ 0; -dD*cos(beta)+dL*sin(beta); -dD*sin(beta)-dL*cos(beta)];
        
        dF(:,i) = [0; dF_private(2); dF_private(3)];

        R_1_4=[in.x(i); -0.25*in.c(i); 0];
        
        
        
        %%%%% NB ATTENZIONE PERCHÈ RxDF E NON DFxR ????????
        
        dM(:,i) = cross(R_1_4, dF_private) + [0.5*in.rho*in.c(i)^2*norm(V)^2*cm; 0; 0];
%         fprintf(f_deb, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", in.x(i), V(1), V(2), V(3), beta*180/pi, AoA*180/pi, cl, cd0, cdi, cm, dL, dD, dF(2,i), dF(3,i), dM(1,i), dM(2,i), dM(3,i) );            

        AoAs(i)=AoA;
        Cls(i)=cl;

    
    end
    
    
% % %     fprintf(f_deb, "\n\n\n MATRICE ROTAZIONE\n\n %f %f %f\n%f %f %f\n%f %f %f\n", Rot_Matr(1,1), Rot_Matr(1,2), Rot_Matr(1,3), Rot_Matr(2,1), Rot_Matr(2,2), Rot_Matr(2,3), Rot_Matr(3,1), Rot_Matr(3,2), Rot_Matr(3,3)  );
     
%     fclose(f_deb);
    
    %integro per trovare le forze risultanti in O
    F=[trapz(in.x,dF(1,:)), trapz(in.x,dF(2,:)), trapz(in.x,dF(3,:))];
    
    M=[trapz(in.x,dM(1,:)), trapz(in.x,dM(2,:)), trapz(in.x,dM(3,:))];


    for i=1:3
        Forces(i)=F(i);
        Forces(i+3)=M(i);
    end
    


    %output
    block.OutputPort(1).Data = Forces;
    block.OutputPort(2).Data = Xo;
    block.OutputPort(3).Data = V_tot_gr;
    block.OutputPort(4).Data = AoAs;
    block.OutputPort(5).Data = Cls;



%     toc