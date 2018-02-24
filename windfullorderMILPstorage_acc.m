close all
dbstop if error

% x=[\omega Pm Pv \omega_r]
Xm = 3.5092;
Xs = 3.5547;
Xr = 3.5859;
Rs = 0.01015;
Rr = 0.0088;
H_D = 4;
p = 4;
rho = 1.225;
Rt =14.4;
Copt = 3.2397e-9*5;
k = 1/45;
wb= 60*2*pi;
ws = 60*2*pi;
D = 1;
H = 4;
ch = 0.3;
g = 0.1;
% R = 3;
R = 3;
Xt = 0.07;
V = 1;
theta = 0;
theta_t = 0;
v_wind = 12;
wr = 72*2*pi;
Pgen = 0.3;
Qset = 0;
Sb = 1000000;
theta_t=0;

lambda=2*k*wr*Rt/p/v_wind;

lambda_i=1/(1/(lambda+0.08*theta_t)-0.035/(theta_t^3+1));

Cp=0.22*(116/lambda_i-0.4*theta_t-5)*exp(-12.5/lambda_i);

% Cp=0.0945;

T0=Xr/ws/Rr;
Asys=[-2.02154129907162,-2.02154129907162,3.37221448651499,3.37221448651499,-35.3284363194502,-15.0845952408218,0.0294957097488978;2.97844442004936,-2.02155557995064,3.37223830902464,3.37223830902464,-37.4796739118547,26.7248890390051,0.0294955683251935;-7.44441285846281e-05,-7.44441285846281e-05,0.000124183250158616,0.000124183250158616,-14.2775861756884,-18.0843676610209,-7.37221036020205e-07;-0.000244069227436211,-0.000244069227436211,5.00040714171155,0.000407141711549698,-41.2543331262190,-20.4786187023088,-2.41702028262211e-06;-0.0181189111750564,-0.0181189111750564,368.957755714042,368.957755714042,-3061.48818901534,-1438.14598197400,0.410520568220463;-219.766019846888,-219.766019846888,-248.822326010879,-248.822326010879,2692.41965430183,-1999.05688381319,-2.78814534669016;-0.000755072597575018,-0.000755072597575018,0.00125956702100636,0.00125956702100636,-113.747049616753,199.607813243406,-0.0262680762490353;];

obj_u = 1; % when set minimizes 1-norm of control signal

ctrl_horizon = 1; % control horizon for MPC

Pd=0.15;
SP.u0.max = 0.5; % max control allowed
SP.u0.min = -0.5; % min control allowed

SP.A = zeros(10,10);

SP.A(1:7,1:7) = Asys;
SP.A(8:10,8:10) = [-D/H/2 ws/2/H 0;
    0 -1/0.3 1/0.3;
    -10/R/2/pi 0 -10];
Csys = [0.404308259814324,0.404308259814324,-0.674442897302999,-0.674442897302999,7.06568726389003,3.01691904816436,0.00400386920820763;]/5;
Dsys = 0.404308259814324/5;

SP.A(8,1:7) =  Csys*ws/2/H;

SP.Bu(:,1) = [2.9785
    2.9784
    -0.0001
    -0.0002
    -0.0181
    -219.7660
    -0.0008
    ws/2/H*Dsys
    0
    0];

SP.Bu(:,2) = [0
    0
    0
    0
    0
    0
    0
    ws/2/H
    0
    0];

SP.Bw = zeros(10,2);
% SP.Bw(8) = -ws/2/H*Pd*ds2;
SP.B = [SP.Bu SP.Bw];

SP.C = eye(10);

SP.Du = zeros(10,2);
SP.Dw = zeros(size(SP.Du));
SP.D = [SP.Du SP.Dw];

SP.u0.A = SP.A;
SP.u0.B = SP.B;
%   SP.u0.Bu = SP.Bu;
%   SP.u0.Bw = SP.Bw;
SP.u0.C = SP.C;
SP.u0.D = SP.D;
%       SP.u0.Du = SP.Du;
%       SP.u0.Dw = SP.Dw;

SP.u0.input_values_human = [];

SP.n = size(SP.A,1);
SP.n_inputs = size(SP.Bu,2);
SP.n_outputs = size(SP.C,1);

SP.x0 = zeros(SP.n,1);
ds2=0.05;
SP.observer_gains = 0.1*eye(3);
SP.pred_hor = 100;
SP.tf = SP.pred_hor*ds2;
SP.pred_known_sz = 2; % We know Goal and one Obstacle from the begining
% of the path planning process
SP.pred_unknown_sz = 0; % We assume we might encounter one more
% Obstacle in future

SP.pred_unknown_sz_orig = SP.pred_unknown_sz;
SP.unknown_pred_flag = 0;
SP.t0 = 0; % initial time
SP.ds = ds2; % time-step
%   SP.phi_orig = '[]a';
%
%     % Unsafe 1
%     ii = 1;
%     SP.pred(ii).str='a';
%     SP.pred(ii).A = [-1 0 0];
%     SP.pred(ii).b = 0.5;
%     SP.phi_goal_1 = '[]a';

%     % Unsafe 1
%     ii = 1;
%     SP.pred(ii).str='a';
%     SP.pred(ii).A = [-1 0 0];
%     SP.pred(ii).b = 0.5;
%     SP.pred(ii).safe = 1;
%     SP.phi_goal_1 = '[]a';
rob_ball_des = 0.217*2*pi; % desired robustness radius
rob_ball_des2 = 6.08*2*pi;
% Unsafe 1
ii = 1;
SP.pred(ii).str='a';
SP.pred(ii).A = [0 0 0 0 0  0 0 -1 0 0; 0 0 0 0 0  0 0 1 0 0;0 0 0 0 0  0 -1 0 0 0; 0 0 0 0 0  0 1 0 0 0];
SP.pred(ii).b = [0.5*2*pi-rob_ball_des;0.5*2*pi-rob_ball_des;10*2*pi-rob_ball_des2;10*2*pi-rob_ball_des2];
SP.pred(ii).safe = 1;
%  SP.phi_goal_1 = '[]a';
%  SP.phi_orig = '[]a';
%  Unsafe 2 

ii = 2;
SP.pred(ii).str='b';
SP.pred(ii).A = [0 0 0 0 0  0 0 -1 0 0; 0 0 0 0 0  0 0 1 0 0];
SP.pred(ii).b = [0.4*2*pi;0.4*2*pi]-rob_ball_des;
SP.pred(ii).safe = 1;
%     SP.phi_goal_2 = '[]_[2,inf]b';
%     % Unsafe 3
%     ii = 3;
%     SP.pred(ii).str='c';
%     SP.pred(ii).A = [0 0 0 0 0  0 -1 0 0 0; 0 0 0 0 0  0 1 0 0 0];
%     SP.pred(ii).b = [10*2*pi;10*2*pi]-rob_ball_des2;
%     SP.pred(ii).safe = 1;
%     SP.phi_goal_3 = '[]_[0,inf]c';

SP.specVars.pred_unknown_sz = 0;
SP.specVars.pred_known_sz = 2;

% SP.pred_unknown_sz_orig = SP.pred_unknown_sz;
% SP.unknown_pred_flag = 0;

SP.specVars.predNum = SP.specVars.pred_unknown_sz+SP.specVars.pred_known_sz;
SP.crit_predSafety=[];
SP.constraints_tmin = [];
SP.constraints_tInd = [];
phiList{1} = strcat('G_[',num2str(0), ...
    ',', num2str(5),']a');

phiList{2} = strcat('G_[',num2str(2), ...
    ',', num2str(5),']b');

% SP.phi_orig = [SP.phi_goal_1 '/\' SP.phi_goal_2];
SP.phiObj = [];
SP.tout=0:0.05:5;
SP.tout=SP.tout';
if ~isempty(phiList)
    phiObj = {};
    for iComp = numel(phiList):-1:1
        phiObj{iComp} = MTLparser(phiList{iComp}, SP.tout, SP.ds);
    end
    SP.phiObj = phiObj;
end

SP.specVars.predIndx = 1:SP.specVars.predNum;

%     if isempty(SP.pred_unknown_sz)
%         SP.pred_num = SP.pred_known_sz;
%     else
%         SP.pred_num = SP.pred_known_sz + SP.pred_unknown_sz; % total number of predicates
%     end

%% -------------------------- System Parameters --------------------------------

SP.times = SP.t0:ds2:SP.tf; % time-points
SP.u0.times = SP.times;

SP.input_length = size(SP.times,2);

SP.return_flag = 0;
SP.extra = zeros(10,1);
SP.extra(8) = -ws/2/H*Pd*ds2;
%     SP.u0.dist_input_values = 0; % disturbance input signal
%     SP.u0.dist_input_values = SP.u0.dist_input_values(ones(SP.n_outputs,1),:);
SP.u0.dist_input_values = 0.0*ones(2,length(SP.times));


SP.u0.input_values_human = zeros(SP.n_inputs, SP.input_length); %ignore this variable ---> needed for some other extensions of this work

% Initialize state variables

SP.states_system = SP.x0; % [w; Pm; Pv]
SP.controller_input_system = 0; % control input to the nonlinear
% system dynamics

%% ------------------------- Optimization Parameters ---------------------------

SP.rob_ball_des = rob_ball_des;
SP.obj_u = obj_u;
SP.ctrl_hor = ctrl_horizon;
SP.hor_length = SP.pred_hor; % simulation horizon

SP.u0.min = SP.u0.min(:,ones(1,SP.hor_length+1)); % minimum control input ---> set in system description code
SP.u0.max = SP.u0.max(:,ones(1,SP.hor_length+1)); % maximum control input

%% ------------------------------ PLOTTING -------------------------------------

SP.pred_actual = SP.pred; % store the actual predicate size
% SP = modify_predicates_rob_ball( SP ); % shrink/bloat predicates depending on the robustness radius

SP.pred_orig = SP.pred;
SP.algoName = 'fmincon'; % 'nlopt'; % 'openopt-ralg'; %

SP.opt_vars.run = 1; % when set to '1' runs the optimization problem
SP.opt_vars.obj_flag = 0; % when set a cost function is to be optimized
SP.opt_vars.obj_u = 1;
SP.opt_vars.col_num = 0;
SP.robBallDes = [];

SP.col_avoid = 0;
SP.objectiveFunc = 'robustness'; % objective is to maximize robustness
SP.constraint_flag = 1; % when set constraints to satisfy temporal robustness
%% -------------------------- RUN OPTIMIZATION ---------------------------------
SP.sol_count=0;
SP.solTime = 0;
argvout_temp = generate_critical_constraints_Single_Agent_MILPpower2(SP);

SP=argvout_temp;
u = argvout_temp.u0.values;
tIndxs = 1:SP.pred_hor+1;
sol_count = 1;
x0 = SP.x0;

SP.dmin_con = -Inf; % initialize robustness radius to -Infinity

%=============================================================================
% Solve for the new trajectory of the system
%=============================================================================

SP.tInd_con = 0; % critical time index
SP.crit_pred = 0; % critical predicate
SP.constraints_tInd = zeros; % stores critical time indices values
SP.crit_predVal = zeros; % stores corresponding critical predicates

u(:,101)=u(:,100);
for i=1:101
    for j=10*i-9:10*i
    us(:,j)=u(:,i);
    end
end
tt=0:0.005:5;
SP.break_loop = 0;

prev_tInd_ind = []; % tracks if a critical predicate at a certain time was added already ---> avoids looping
cc=0.5;
SP.constraints_tInd(sol_count) = SP.tInd_con;
SP.crit_predVal(sol_count) = SP.crit_pred;

plot(tt, us(1,1:1001),'LineWidth',2,'color','k')
axis([0 5 -cc cc]);
xlabel('t (s)','fontsize',20)
ylabel('u^w','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tt, us(2,1:1001),'LineWidth',2,'color','k')
axis([0 5 -cc cc]);
xlabel('t (s)','fontsize',20)
ylabel('u^s','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(SP.times, SP.yout(1:101,8)/2/pi,'LineWidth',2,'color','k')
axis([0 5 -0.6 0.1]);
xlabel('t (s)','fontsize',20)
ylabel('\Deltaf (Hz)','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')
xx1=SP.yout(1:101,:);
tspan=SP.times;

figure;
plot(tspan, xx1(:,7)/2/pi,'LineWidth',2,'color','k')
axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\Deltaf_r (Hz)','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tspan, xx1(:,9),'LineWidth',2,'color','k')
axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\DeltaP_m','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tspan, xx1(:,10),'LineWidth',2,'color','k')
 axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\DeltaP_v','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tspan, xx1(:,1),'LineWidth',2,'color','k')
 axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\Deltax_1','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tspan, xx1(:,2),'LineWidth',2,'color','k')
 axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\Deltax_2','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tspan, xx1(:,3),'LineWidth',2,'color','k')
 axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\Deltax_3','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tspan, xx1(:,4),'LineWidth',2,'color','k')
 axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\Deltax_4','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tspan, xx1(:,5),'LineWidth',2,'color','k')
 axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\DeltaEq','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

figure;
plot(tspan, xx1(:,6),'LineWidth',2,'color','k')
 axis([0 5 -inf inf]);
xlabel('t (s)','fontsize',20)
ylabel('\DeltaEd','fontsize',20)
set(gca,'linewidth',2,'fontsize',20,'fontname','Times');
set(gcf,'color','white')

