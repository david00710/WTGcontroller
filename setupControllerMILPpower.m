function [OptControllerHeuristics, FeasController] = setupControllerMILPpower(SP)
% [OptController, FeasController] = setupControllerMILP(SP) setups MILP controllers : one for finding a feasible
% solution and the other to find the optimal solution

%%  define variables to be optimized
u = sdpvar(repmat(SP.n_inputs,1, SP.input_length),ones(1,SP.input_length)); % control signal variable
x = sdpvar(repmat(SP.n,1, SP.input_length + 1),ones(1, SP.input_length + 1)); % state space variable
y = sdpvar(repmat(SP.n_outputs,1, SP.input_length + 1),ones(1,SP.input_length + 1)); % output variable
w = sdpvar(repmat(SP.n_inputs,1, SP.input_length),ones(1,SP.input_length)); % disturbance signal variable

rob_val = sdpvar(1,1); % robustness degree

con_active = sdpvar(SP.specVars.predNum + SP.opt_vars.col_num, SP.input_length + 1); % tracks which constraints to be set to active
slacks = sdpvar(SP.specVars.predNum + SP.opt_vars.col_num, SP.input_length + 1); % tracks which constraints leads to infeasibility

d = {}; % bigM variable required for placing constraints to avoid unsafe sets ---> see below in the constraints section
M = 1000; % bigM value
x_cur = sdpvar(SP.n,1);
rem_hor_length = sdpvar(1,1);          

% account for unknown predicates -----------------------------------
% ------------------------------------------------------------------
for ii = 1:SP.specVars.predNum + SP.opt_vars.col_num % SP.pred_known_sz
    pred_A(ii).vals = sdpvar(size(SP.pred(ii).A,1), size(SP.pred(ii).A,2), 'full');
    pred_b(ii).vals = sdpvar(size(SP.pred(ii).b,1), size(SP.pred(ii).b,2), 'full');
end

%   for ii = SP.pred_known_sz+1:SP.predNum
%     pred_A(ii).vals = sdpvar(size(SP.pred(ii).A,1), SP.n_outputs);
%     pred_b(ii).vals = sdpvar(size(SP.pred(ii).A,1), 1);
%   end

%% following variables are required for implementing the adversarial games scenario only
if SP.col_avoid ~= 0
    
    other_y = sdpvar(repmat(SP.n_outputs,1, SP.input_length),ones(1,SP.input_length)); % output variable for the other agent
    
    %       col_active = sdpvar(1, SP.input_length); % tracks which collision avoidance constraints to be set to active
    
    % --> other_SP.traj_saved(2:end,t_crit_obj)
    %     delta_bound = sdpvar(1,1); % bound on the separation between the adversary and the system
    %   else
    %       other_y = sdpvar(1,1);
end

%% initialize constraints and objective


% if SP.obj_u
uVec = [u{:}];
constraints = []; % [-.1 <= diff([pastu u{:}]) <= .1];

if isempty(SP.robBallDes)
      lambda=100;
  objective = uVec(1,:)*uVec(1,:)'+lambda*uVec(2,:)*uVec(2,:)';
%     objective = sum(sum(abs(uVec)))/length(uVec(:)) - rob_val;
    objectiveFeas = norm(slacks(:), 1);
    % SP.obj_u*uVec(:)'*uVec(:); % %  SP.R*norm([u{:}],1); %  % minimize 1-norm of control input
    constraints = [constraints, 0 <= rob_val <= 5];
else
    objective = sum(sum(abs(uVec)));
    objectiveFeas = sum(slacks(:)); % norm(slacks(:), 1); % 
    constraints = [constraints, (rob_val == 0):'robValue']; % SP.robBallDes];
end


if isempty(SP.u0.input_values_human)
    SP.u0.input_values_human = zeros(size(u));
end

constraints = [constraints, (x{1} == x_cur):'initialState'];


for k = 1:SP.input_length
    %   objective = 0; % objective + SP.obj_u*norm(SP.R*u{k},1); % 1-norm of control signal
    
    %     % minmize separation between last known trajectory (xout) and currently optimized trajectory (x)
    % %     if SP.opt_vars.obj_flag
    % %       objective = objective + norm(SP.C*(x{k}-SP.xout(k,:)'),1); %
    % %     end
    
    %   uvals = u{k}; % + SP.u0.input_values_human(:,k); %  + u_last(:,k);
    % add system dynamics as constraints
    constraints = [constraints, (x{k+1} == SP.tf_disc.A*x{k}+SP.tf_disc.B*[u{k}; w{k}]+SP.extra):'xDynamics']; % uvals];
    constraints = [constraints, (y{k} == SP.tf_disc.C*x{k}+SP.tf_disc.D*[u{k}; w{k}]):'yDynamics']; % uvals];
    
    % add bounds on control input signal as constraints
    if k<SP.input_length
    constraints = [constraints, (abs(u{k+1}(1)-u{k}(1)) <=0.05):'u1Bound'];
    constraints = [constraints, (abs(u{k+1}(2)-u{k}(2)) <=0.05):'u2Bound'];
    end
    constraints = [constraints, (SP.u0.min(:,k) <= u{k} <= SP.u0.max(:,k)):'uBound'];
    %     constraints = [constraints, SP.u0.min <= u{k} <= SP.u0.max];
    if k > value(rem_hor_length)
        constraints = [constraints, (u{k} == zeros(SP.n_inputs,1)):'uZeros'];
    end
    
end                       

% add the constraint for output signal at the last time point
constraints = [constraints, (y{end} == SP.tf_disc.C*x{end}+SP.tf_disc.D*[u{end}; w{end}]):'yEnd']; % uvals];

%% encode all the constraints
sz_d = 0;

for ii = 1:SP.specVars.predNum
    
    if SP.pred(ii).safe == -1 % predicates to be avoided
        
        for jj = 2:SP.input_length+1
            
            sz_d = sz_d + 1;
            
            if ii > SP.specVars.pred_known_sz
                sz_pred = 4;
            else
                sz_pred = size(SP.pred(ii).b,1); % number of hyperplanes constructing the predicate
            end
            
            d{sz_d} = binvar(sz_pred,1); % update the variable holding the bigM variables
            
            % add constraint such that the state is outside of atleast one of the hyperplanes
            constraints = [constraints, (sum(d{sz_d}(:,1))*con_active(ii,jj) <= (sz_pred-1)*con_active(ii,jj)):'dConstraint'];
            
            for kk = 1:sz_pred
                constraints = [constraints, ((pred_A(ii).vals(kk,:)*y{jj} ...
                    - pred_b(ii).vals(kk) - rob_val)*con_active(ii,jj) + slacks(ii, jj) >= -M*d{sz_d}(kk,1)*con_active(ii,jj)):'AvoidPred'];
            end
        end
        
    else % predicates to be visited
        
        for jj = 2:SP.input_length+1
            constraints = [constraints, (pred_A(ii).vals*y{jj}*con_active(ii,jj) - slacks(ii, jj) <= (pred_b(ii).vals - rob_val)*con_active(ii,jj)):'SafePred'];
         %  constraints = [constraints, SP.pred(ii).A*x{jj} <= SP.pred(ii).b + con_active(ii,jj)];
        end
        
    end
end


if SP.opt_vars.col_num > 0
    
    for ii = SP.specVars.predNum+1:SP.specVars.predNum+SP.opt_vars.col_num
        
        % predicates to be avoided
        
        for jj = 2:SP.input_length+1
            
            sz_d = sz_d + 1;
            
            %         if ii > SP.specVars.pred_known_sz
            sz_pred = 4;
            %         else
            %           sz_pred = size(SP.pred(ii).b,1); % number of hyperplanes constructing the predicate
            %         end
            
            d{sz_d} = binvar(sz_pred,1); % update the variable holding the bigM variables
            
            % add constraint such that the state is outside of atleast one of the hyperplanes
            constraints = [constraints, sum(d{sz_d}(:,1))*con_active(ii,jj) <= (sz_pred-1)*con_active(ii,jj)];
            
            for kk = 1:sz_pred
                constraints = [constraints, (pred_A(ii).vals(kk,:)*y{jj} ...
                    - pred_b(ii).vals(kk))*con_active(ii,jj) >= -M*d{sz_d}(kk,1)*con_active(ii,jj)];
            end
        end
        
    end
end

%% Add constraints so that the closest distance between the adversary and system is small for adversary and large for system if required
% this is only for adversarial games scenario

if SP.col_avoid ~= 0
    
    sz_pred = 4;
    
    % add constraints so that either the minimum distance between two trajectories
    % are smaller than a bound or greater than a bound depending on whether the
    % object is the adversary or the system
    
    for ii = 1:SP.opt_vars.col_num
        for jj = 2:SP.input_length+1
            sz_d = sz_d + 1;
            
            d{sz_d} = binvar(sz_pred,1); % update the variable holding the bigM variables
            
            % add constraint such that the state is outside of atleast one of the hyperplanes
            constraints = [constraints, sum(d{sz_d}(:,1))*con_active(ii + SP.specVars.predNum, jj) <= (sz_pred-1)*con_active(ii + SP.specVars.predNum, jj)];
            
            for kk = 1:sz_pred
                constraints = [constraints, (SP.pred(ii + SP.specVars.predNum).A(kk,:)*(y{jj} - other_y{jj})...
                    - SP.pred(ii + SP.specVars.predNum).b(kk) - SP.safe_rad)*con_active(ii + SP.specVars.predNum, jj) >= -M*d{sz_d}(kk,1)*con_active(ii + SP.specVars.predNum,jj)];
            end
        end
        
        % constraints = [constraints, norm((y{k} - other_y{k}),1)*SP.col_avoid*col_active(k) <= SP.safe_rad*SP.col_avoid*col_active(k)];
        
    end
    
    % define what the controller inputs are
    parametersIn = {x_cur, con_active, rem_hor_length, [w{:}], pred_A.vals, pred_b.vals, [other_y{:}]}; % , col_active}; % {x{1}, x_crit_other, delta_bound};
    
    %     constraints_tInd = [SP.input_length+1 SP.tInd_obj]; % keeps track of time points where constraints are placed
    %     indx = 2; % keeps track of number of time points where constraints are placed
    
else
    
    parametersIn = {x_cur, con_active, rem_hor_length, [w{:}], pred_A.vals, pred_b.vals}; % [con_active{:}]};
    
end


constraintsFeas = [constraints, (slacks >= 0):'slacks'];
constraintsOptHeuristics = [constraints, (slacks == 0):'slacks'];

% define what the controller outputs are
solutionsOut = {[u{:}], [x{:}], [y{:}], rob_val, slacks};


%% define settings

% verbose = 2 ---> gives detailed iteration wise results ---> good for debugging
% change solvername to use a different solver than gurobi
% if no solver option is provided yalmip uses the default in-built one

ops = sdpsettings('verbose', 1, 'solver', 'gurobi', 'cachesolvers', 1, 'savesolveroutput', 1, 'savesolverinput', 1, 'saveyalmipmodel', 1 );
OptControllerHeuristics = optimizer(constraintsOptHeuristics, objective, ops, parametersIn, solutionsOut);
% optimal controller with heuristics
FeasController = optimizer(constraintsFeas, objectiveFeas, ops, parametersIn, solutionsOut);
% feasibility controller

end