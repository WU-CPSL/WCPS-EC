function [sys,x0,str,ts,simStateCompliance] = ControllersfunV_P(t,x,u,flag,Kv,Ki,Kd,delta_t,delta_r,B,J,Km,Q,R,S)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes();

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u,Kv,Ki,Kd,delta_t,delta_r,B,J,Km,Q,R,S);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes()
%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 7;
%x(1) U after network
%x(2) Counter
sizes.NumOutputs     = 1;
sizes.NumInputs      = 3; 
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%

x0  = [0,0,0,0,0,1,0];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];
% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,Kv,Ki,Kd,delta_t,delta_r,B,J,Km,Q,R,S)
if (rem(x(6),delta_t/delta_r)== 0)
w_ref = u(1);
w = u(2);
ecurrent = u(3);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% %%%%% PI controller %%%%%%%%
deltaP=Kv*(ecurrent-x(5));
deltaI=Ki*ecurrent*delta_t; %sample time 
deltaD=Kd*(ecurrent-x(5) - x(5)+x(7))/delta_t;
x(3)=x(2)+deltaP+deltaI+deltaD;

x(1)=w_ref;
x(4)=w;
x(7)=x(5);
x(5)=ecurrent;
x(2)=x(3);


%% %%%%% MPC controller %%%%%%%%%%
% Nstates=1;      %number of states
% Ninputs=1;      %number of inputs
% Horizon = 7;
% 
% MMQ=Q*eye(Nstates);
% MMR=R*eye(Ninputs);
% MMS=S*eye(Nstates);
% 
% Oidxs=1;
% Nopt=Nstates*Horizon+Ninputs*(Horizon-1);
% 
% SSMA=1-delta_t*B/J;
% SSMB=delta_t*Km/J;
% SSMC=1;
% 
% H=zeros(Nopt,Nopt);
% f=zeros(Nopt,1);
% 
% 
% for k = 1:Horizon-1
%   H( (k-1)*Nstates + Oidxs, ...
%      (k-1)*Nstates + Oidxs ) = 2 * SSMC * MMQ * SSMC';
%   H( Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs), ...
%      Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs) ) = 2 * MMR;
%   f( (k-1)*Nstates + (1:Nstates) ) = - 2 * w_ref * SSMC * MMQ;
% end
% H( (Horizon-1)*Nstates + Oidxs, ...
%    (Horizon-1)*Nstates + Oidxs ) = 2 * SSMC * MMS * SSMC';
% f( (Horizon-1)*Nstates + (1:Nstates) ) = - 2 * w_ref * SSMC * MMS;
% 
% Aeq = zeros( Nstates * Horizon, Nopt );
% beq = zeros( Nstates * Horizon, 1 );
% 
% for k = 1:(Horizon-1)
%   Aeq( (k-1)*Nstates + (1:Nstates), ...
%        (k-1)*Nstates + (1:Nstates) ) = -SSMA;
%   Aeq( (k-1)*Nstates + (1:Nstates), ...
%            k*Nstates + (1:Nstates) ) = eye(Nstates);
%   Aeq( (k-1)*Nstates + (1:Nstates), ...
%        Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs) ) = -SSMB;
% end
% Aeq( (Horizon-1)*Nstates + (1:Nstates), ...
%                             1:Nstates ) = eye(Nstates);
% beq( (Horizon-1)*Nstates + (1:Nstates) ) = [w];
% 
% lb = zeros( Nopt, 1 );
% ub = zeros( Nopt, 1 );
% for k = 1:Horizon
%     lb( (k-1)*Nstates + (1:Nstates) ) = -inf;
%     ub( (k-1)*Nstates + (1:Nstates) ) = inf;
% end
% 
% for k = 1:Horizon-1
%     ub( Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs) ) = 0.9/Km;
%     lb( Nstates*Horizon + (k-1)*Ninputs + (1:Ninputs) ) = -0.9/Km;
% 
% end
% 
% 
% z=zeros(Nopt,1);
% options = optimoptions('quadprog',...
%     'Algorithm','interior-point-convex','Display','off');
% z=quadprog(H,f,[],[],Aeq,beq,lb,ub,[],options);
% x(3) = z(Nstates*Horizon + 1);



% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
x(6)=x(6)+1;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys = [x];
% end mdlUpdate
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
sys = [x(3)];
% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;
% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];
% end mdlTerminate
