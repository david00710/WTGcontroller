function [SP, break_loop_index, lastInfeasibleIdx] = solve_local_MILP_SysObj_AllPaths( SP, global_sol_count, lastInfeasibleIdx, obj )
% [SP, break_loop_index] = solve_agent_optimization( SP )
% solves each agents' MILP problem locally
%  [SP, break_loop_index, lastInfeasibleIdx] = solve_local_MILP_SysObj_AllPaths( SP, 0, lastInfeasibleIdx );
% SP.sol_count = SP.sol_count + 1;          
% dbstop;

%------------------------------------------------------------------
% --------------------- Run Optimization --------------------------
%------------------------------------------------------------------
SP.solTime = 0;
if SP.col_avoid
    [solutions,diagnostics, ~, ~, ~, details] = SP.OptControllerHeuristics{{SP.x_cur(:) , SP.con_active , SP.rem_input_length, ...
        SP.u0.dist_input_values, SP.pred.A, SP.pred.b, SP.other_yout'}}; % , SP.col_active}};
else
    [solutions,diagnostics, ~, ~, ~, details] = SP.OptControllerHeuristics{{SP.x_cur(:) , SP.con_active , SP.rem_input_length, ...
        SP.u0.dist_input_values, SP.pred.A, SP.pred.b}}; % , SP.col_active}};
end

SP.solTime = SP.solTime + details.solvertime;

%% Handle infeasible constraint
if diagnostics == 1 || diagnostics == 12 || diagnostics == 15
    
    [solutionsFeas, diagnosticsFeas, ~, ~, ~, detailsFeas] = SP.FeasController{{SP.x_cur(:) , SP.con_active , SP.rem_input_length, SP.u0.dist_input_values, SP.pred.A, SP.pred.b}}; % , SP.col_active}};
    SP.solTime = SP.solTime + detailsFeas.solvertime;
    
    %     if diagnosticsFeas == 1 || diagnosticsFeas == 12 || diagnosticsFeas == 15
    %         if SP.getExtData
    %             cleanup_vrep(obj.vrep, obj.ID);
    %         end
    %         error('The problem is infeasible');
    %     end
    slacks = solutionsFeas{end};
    
    infeasibleIdx = find(slacks ~= 0);
    
%     if isequal(infeasibleIdx , lastInfeasibleIdx) || diagnosticsFeas == 1 || diagnosticsFeas == 12 || diagnosticsFeas == 15
%         % if you again get back the same set of infeasible constraints then the problem is infeasible.
%         printf('\nThe problem is infeasible. \n');
%         break_loop_index = 1;
%         
%         if SP.getExtData
%             cleanup_vrep(obj.vrep, obj.ID);
%         end
%         
%         return;
%     end
    
    lastInfeasibleIdx = infeasibleIdx;
    
    [SP, break_loop_index] = handleInfeasibleConstraintAssignments(SP, slacks, infeasibleIdx);
    if break_loop_index
        return
    end
    
else
    
    
    %% Get new solution values
    SP.u0.values = solutions{1};
    SP.xout = solutions{2}';
    % SP.yout_disc = solutions{3}';
    
    %   compute continuous time trajectory
    %   SP.xout_mpc = dynamics(SP.hor_length*SP.ds, SPx_cur, SP.u0.values + SP.u0.last, SP.A, SP.B, [], SP.ds)'; % , di)
    
    %   SP.xout_mpc = SP.xout_disc;
    
    % SP.yout = solutions{3}';
    SP.yout(SP.iter+1:end,:) = solutions{3}(:,2:end-(SP.iter-1))';
    % SP.yout(SP.iter+1:end,:) = solutions{3}(:,1:SP.rem_input_length)';

    %------------------------------------------------------------------
    % --------------------- Compute Robustness ------------------------
    %------------------------------------------------------------------
    
    
    % only use the relevant portion of the trajectory, i.e, till the given
    % trajectory end time
    
    nTemporals = numel(SP.phiObj);
    for iComp = nTemporals:-1:1
        [~, tminConComps(iComp), dminConComps(iComp), ~, critPredComps(iComp)] = ...
            ... staliro_distance_components(SP.phiObj{iComp}, SP.pred, SP.yout, SP.tout);
            getRobustnessMTL(SP.phiObj{iComp}, SP.pred, SP.yout, SP.tout, ...
            SP.constraints_tmin(SP.crit_predSafety == 1), SP.crit_predVal(SP.crit_predSafety == 1));
    end
    
    [dminCon, idx] = min(dminConComps);
    tminCon = tminConComps(idx);
    critPred = critPredComps(idx);
    
    %------------------------------------------------------------------
    % -----------------  Find critical constraints --------------------
    %------------------------------------------------------------------
    
    SP.tInd_con = round(tminCon/SP.ds)+1 -(SP.iter-1);
    SP.prev_tInd_ind = find(SP.constraints_tInd == SP.tInd_con);
    
    
    %% add constraints corresponding to the critical predicates
    if ((isempty(find(SP.crit_predVal(SP.prev_tInd_ind) == critPred, 1))) ...
            && (tminCon > (SP.iter-1)*SP.ds && dminCon < 0 && ...
            abs(dminCon) > 1e-6) && toc(SP.iter_time) <= SP.specVars.ds)
        % (dminCon < 0 && tminCon ~= 0))
        
        SP.con_active(critPred, SP.tInd_con) = 1; 
        
        SP.sol_count = SP.sol_count + 1;
    
        SP.constraints_tInd(SP.sol_count+global_sol_count) = SP.tInd_con;
        SP.constraints_tmin(SP.sol_count+global_sol_count) = tminCon;
        SP.crit_predVal(SP.sol_count+global_sol_count) = critPred;
        SP.crit_predSafety(SP.sol_count+global_sol_count) = SP.pred(critPred).safe;
       
%         SP.con_active=[0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0;0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1,1,0];
%         SP.constraints_tInd=[14,41,62,81,100,24,51,70,89,31,56,46,75,10,18,66,101,59,12,27,21,53,48,94,85,16,34,44,78,72,64,68,11,29,49,54,57,60,13,19,22,15,25,32,43,91,83,73,87,17,101,76,79,20,23,26,28,33,30,50,52,47,55,58,45,61,63,65,67,42,69,96,92,71,74,77,80,82,84,86,88,90,93,95,35];
%         SP.constraints_tmin=[0.650000000000000,2,3.05000000000000,4,4.95000000000000,1.15000000000000,2.50000000000000,3.45000000000000,4.40000000000000,1.50000000000000,2.75000000000000,2.25000000000000,3.70000000000000,0.450000000000000,0.850000000000000,3.25000000000000,5,2.90000000000000,0.550000000000000,1.30000000000000,1,2.60000000000000,2.35000000000000,4.65000000000000,4.20000000000000,0.750000000000000,1.65000000000000,2.15000000000000,3.85000000000000,3.55000000000000,3.15000000000000,3.35000000000000,0.500000000000000,1.40000000000000,2.40000000000000,2.65000000000000,2.80000000000000,2.95000000000000,0.600000000000000,0.900000000000000,1.05000000000000,0.700000000000000,1.20000000000000,1.55000000000000,2.10000000000000,4.50000000000000,4.10000000000000,3.60000000000000,4.30000000000000,0.800000000000000,5,3.75000000000000,3.90000000000000,0.950000000000000,1.10000000000000,1.25000000000000,1.35000000000000,1.60000000000000,1.45000000000000,2.45000000000000,2.55000000000000,2.30000000000000,2.70000000000000,2.85000000000000,2.20000000000000,3,3.10000000000000,3.20000000000000,3.30000000000000,2.05000000000000,3.40000000000000,4.75000000000000,4.55000000000000,3.50000000000000,3.65000000000000,3.80000000000000,3.95000000000000,4.05000000000000,4.15000000000000,4.25000000000000,4.35000000000000,4.45000000000000,4.60000000000000,4.70000000000000,1.70000000000000];
%         SP.crit_predVal=[1,2,2,2,2,1,2,2,2,1,2,2,2,1,1,2,2,2,1,1,1,2,2,2,2,1,1,2,2,2,2,2,1,1,2,2,2,2,1,1,1,1,1,1,2,2,2,2,2,1,1,2,2,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1];
%         SP.crit_predSafety=[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1];

%         SP.con_active(2, 41) = 1; 
%         SP.constraints_tInd(2) = 41;
%         SP.constraints_tmin(2) = 2;
%         SP.crit_predVal(2) = 2;
%         SP.crit_predSafety(2) = 1;

        break_loop_index = 0;
    else
        
        break_loop_index = 1;
        
    end
  
%     SP.dminCon = dminCon;
    
end

end

