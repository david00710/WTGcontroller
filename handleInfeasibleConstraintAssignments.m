function [SP, break_loop_index] = handleInfeasibleConstraintAssignments(SP, slacks, infeasibleIdx)


break_loop_index = 0;

% find the predicate and time pairs causing infeasibility and deactivate the infeasible constraint

for iPred = 1:numel(infeasibleIdx)
    
%     infeasibilityVal = max(slacks(infeasibleIdx));
%     [infeasiblePred, infeasibleTimeIdx] = ind2sub(size(slacks), find(slacks == infeasibilityVal));
    
    [infeasiblePred, infeasibleTimeIdx] = ind2sub(size(slacks), infeasibleIdx(iPred));
    SP.infeasibleTimeIdx = [SP.infeasibleTimeIdx infeasibleTimeIdx];
    
    % deactivate the corresponding assignment
    SP.con_active(infeasiblePred, infeasibleTimeIdx) = 0;
    
    % remove the corresponding elements from the storage variables
    tmpId = SP.constraints_tInd == infeasibleTimeIdx;
    SP.crit_predVal(tmpId) = [];
    SP.crit_predSafety(tmpId) = [];
    SP.constraints_tmin(tmpId) = [];
    SP.constraints_tInd(tmpId) = [];
    
    % modify the phiObjs to take the infeasibile constraint into account when finding critical predicate
    for iObj = 1:numel(SP.phiObj)
        phiObj = SP.phiObj{iObj};
        for iComp = 1:phiObj.numComps
%             if phiObj.predId == infeasiblePred && any(phiObj.timeIdx{iComp} == infeasibleTimeIdx)
              if ismember(infeasiblePred,phiObj.predId) && any(phiObj.timeIdx{iComp} == infeasibleTimeIdx)
                % infeasible constraint belongs to this component of this phiObj
                
                if strcmp(phiObj.type, 'alw')
                    
                    % for always all timepoints should be true : clearly that is not possible here : so infeasible
                    % problem            
                    
%                     fprintf('\n The problem is infeasible.\n');
%                     break_loop_index = 1;                                        
%                     return;
                      phiObj.timeIdx{iComp}(tmpId) = [];
                else
                    
                    tmpTimeIdx = 1:numel(phiObj.timeIdx{iComp});
                    tmpId = tmpTimeIdx(phiObj.timeIdx{iComp} == infeasibleTimeIdx);
                    
                    if strcmp(phiObj.type, 'ev')
                        
                        % remove the time from its timeIdx variable
                        phiObj.timeIdx{iComp}(tmpId) = [];
                        
                    elseif strcmp(phiObj.type, 'ev-alw')
                        
                        % find which subComps the infeasible time belong to and remove that subComp, since we need each
                        % element (time) of the subComp to be true
                        
                        numSubComps = size(phiObj.subTimeIdx, 1);
                        for iSubComp = numSubComps:-1:1
                            if any(phiObj.subTimeIdx(iSubComp, :) ==  tmpId)
                                phiObj.subTimeIdx(iSubComp, :) = [];
                            end
                        end
                        
                    elseif strcmp(phiObj.type, 'alw-ev')
                        
                        numSubComps = size(phiObj.subTimeIdx, 1);
                        for iSubComp = numSubComps:-1:1
                            tmpVal = phiObj.subTimeIdx(iSubComp, :);
                            % hacky way to remove the time without changing the size of the matrix
                            tmpVals2 = tmpVal(tmpVal ~= tmpId);
                            tmpVal(tmpVal ==  tmpId) = tmpVal(tmpVals2(1));
                            phiObj.subTimeIdx(iSubComp, :) = tmpVal;
                        end
                        
                    end
                end
            end
        end
        SP.phiObj{iObj} = phiObj;
    end
    
end

end

