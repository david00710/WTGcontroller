function argvout = generate_critical_constraints_Single_Agent_MILPpower2(SP)

yalmip('clear')

%% Discretize the system
% SP.ss_cont= ss(SP.A, SP.B, SP.C, SP.D);
% SP.tf_disc = c2d(SP.ss_cont, SP.ds);
% 
% %% Setup the controller and the optimization process
% 
% SP.Q = eye(SP.n);
% SP.R = 1;
% 
% sw = tic;
% % SP.controller = setup_MILP_controller(SP);
% [SP.OptControllerHeuristics, SP.FeasController] = setupControllerMILPpower(SP);
% 
% OptControllerHeuristicspower = SP.OptControllerHeuristics;
% FeasControllerpower = SP.FeasController;
% sprintf('Time taken to encode controllers are : %d', toc(sw))
% load('OptControllerHeuristics0.mat');
% load('C:\Users\zhe xu\Dropbox\adhspaper\robot\newtoolbox\OptControllerHeuristic0.mat')
% load('C:\Users\zhe xu\Dropbox\adhspaper\robot\newtoolbox\FeasController0.mat');
% SP.OptControllerHeuristics = OptControllerHeuristic0;
% SP.FeasController = FeasController0;

load('C:\Users\zhe xu\Dropbox\wind\WTG\WTGcode\OptControllerHeuristicspower_acc.mat')
load('C:\Users\zhe xu\Dropbox\wind\WTG\WTGcode\FeasControllerpower_acc.mat');
SP.OptControllerHeuristics = OptControllerHeuristicspower;
SP.FeasController = FeasControllerpower;

SP.x_cur = SP.x0;
SP.u0.values = zeros(SP.n_inputs, SP.input_length);
SP.yout = zeros(SP.input_length + 1, SP.n_outputs);
SP.specVars.ds = 1000; % limits execution time of running each iteration of MILP

%% Initialize other variables
SP.t0 = 0;
SP.tInd_con = 0;
SP.crit_pred = 0;
SP.tminCon = -1;
SP.dminCon = -Inf;
SP.constraints_tInd = zeros;
SP.crit_predVal = zeros;

%% Set constraints and return if all constraints are to be used

% set all constraints to be inactive ---> they are made active as required
SP.con_active = zeros(SP.specVars.predNum, SP.input_length + 1); % bigM*ones(size(SP.pred,2), SP.hor_length+1); %

SP.rem_input_length = SP.input_length;
SP.iter = 1;
SP.yout(1,:) = SP.x_cur(1:SP.n_outputs);

%% Start the main optimization procedure

% sw = tic;
break_loop_index = 0;
SP.iter_time = tic;
lastInfeasibleIdx = 0;

if isempty(SP.phiObj)
    
    
    while ~ break_loop_index
        
        [SP, break_loop_index, lastInfeasibleIdx] = solve_local_MILP_SysObj( SP, 0, lastInfeasibleIdx );
        
    end
    
else
    
    while ~ break_loop_index
        
        [SP, break_loop_index, lastInfeasibleIdx] = solve_local_MILP_SysObj_AllPaths( SP, 0, lastInfeasibleIdx );
        
    end
    
end

argvout = SP;

end


