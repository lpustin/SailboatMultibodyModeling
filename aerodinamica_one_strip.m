function aerodinamica_one_strip(block)
% Level-2 MATLAB file S-Function for continuous time variable step demo.
%   Copyright 1990-2009 The MathWorks, Inc.
%   $Revision: 1.1.6.2 $

setup(block);

function setup(block)

%% Register number of dialog parameters
block.NumDialogPrms = 1;
%% Register number of input and output ports

block.NumInputPorts  = 4;
block.NumOutputPorts = 3;
%% Setup functional port properties to dynamically inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

block.InputPort(1).Dimensions        = 4; % quatern
block.InputPort(1).DirectFeedthrough = false;

block.InputPort(2).Dimensions        = 3; % w
block.InputPort(2).DirectFeedthrough = false;

block.InputPort(3).Dimensions        = 3; % Xo
block.InputPort(3).DirectFeedthrough = false;

block.InputPort(4).Dimensions        = 3; % Vo
block.InputPort(4).DirectFeedthrough = false;

block.OutputPort(1).Dimensions       = 3; % Fx Fy M
block.OutputPort(2).Dimensions       = 3; % L D M
block.OutputPort(3).Dimensions       = 1; %AoA






%% Set the block simStateCompliance to default (i.e., same as a built-in block)

block.SimStateCompliance = 'DefaultSimState';

%% Set up the continuous states.

block.NumContStates = 0;

%% Set block sample time to continuous

block.InputPort(1).SampleTime =[0 0];
block.InputPort(1).SamplingMode = 'Sample';

block.InputPort(2).SampleTime =[0 0];
block.InputPort(2).SamplingMode = 'Sample';

block.InputPort(3).SampleTime =[0 0];
block.InputPort(3).SamplingMode = 'Sample';

block.InputPort(4).SampleTime =[0 0];
block.InputPort(4).SamplingMode = 'Sample';

block.OutputPort(1).SampleTime = [0 0];
block.OutputPort(1).SamplingMode = 'Sample';
block.OutputPort(2).SampleTime = [0 0];
block.OutputPort(2).SamplingMode = 'Sample';
block.OutputPort(3).SampleTime = [0 0];
block.OutputPort(3).SamplingMode = 'Sample';



%% Register methods
block.RegBlockMethod('Outputs',                 @Output);
block.RegBlockMethod('SetInputPortSampleTime', @SetInputPortSampleTime);
block.RegBlockMethod('SetOutputPortSampleTime', @SetOutputPortSampleTime);

function Output(block)
%     tic

    %parametri
    in=block.DialogPrm(1).Data; %struttura par_main

    %Ingressi
    quatern=block.InputPort(1).Data;
    w=block.InputPort(2).Data;
    Xo=block.InputPort(3).Data;
    Vo=block.InputPort(4).Data;


    
    %controllo sui quaternioni
    problema_iniziale=[0;0;0;0];
    
    if isequal(quatern, problema_iniziale)
        disp('PROBLEMA INIZIALE QUATERN 0')
        quatern=[1;0;0;0];
    end


    Rot_Matr=quat2rotm(quatern'); %dal SR pala corpo al SR terra

    if in.isBiphase==true
        Xo_gr=Rot_Matr*Xo;
        euls=quat2eul(quatern');
        roll=euls(2);
        r_imm=Xo_gr(3)/cos(roll);
        if r_imm<0
            r_imm=0;
        end
        if r_imm>in.span
            r_imm=in.span;
        end    
        mean_chord=in.S/in.span;
        S=in.S*(in.span-r_imm)/in.span;
        porz_imm=S/in.S;
    else
        S=in.S;
        
    end
    
    

    
    V_w=Rot_Matr'*[in.V_wind; 0; 0]; %porto la V_wind nel SR corpo

%     Vapp = Vo -cross(w,in.AeroCentr) + V_w; %velocità al CA dovuta al moto del corpo
    Vapp = Vo  + V_w; %velocità al CA dovuta al moto del corpo

        
%     disp(['Vapp=',num2str(Vapp'),' V_w=',num2str(V_w'),' Vo=',num2str(Vo')])

    AoA = atan2( -Vapp(2), Vapp(1) ); %angolo tra il vento e SR corpo al root
    
    if AoA>25*pi/180
        AoA=25*pi/180;
    end
    if AoA<-25*pi/180
        AoA=-25*pi/180;
    end  
    
    L=0.5*in.rho*S*norm(Vapp)^2*in.cl_alpha*AoA;
    cd=(in.cl_alpha*AoA)^2/(pi*in.AR*in.e)+in.cd0;
    D=0.5*in.rho*S*norm(Vapp)^2*cd;
    M=0.5*in.rho*S*norm(Vapp)^2*in.cm_alpha*AoA;
    
    
    Fx=-L*sin(AoA)+D*cos(AoA);
    Fy=-L*cos(AoA)-D*sin(AoA);

   
    %output
    block.OutputPort(1).Data = [Fx;Fy;M];
    block.OutputPort(2).Data = [L;D;M];
    block.OutputPort(3).Data = AoA;




%     toc