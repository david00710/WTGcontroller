function [tmin, dmin] = getCriticalValuesMTLpower(phiTimeIdx, phiSubTimeIdx, phiType, phiNot, traj, predIdx, predicates, ...
    timeStamps, varargin)
% [tmin, dmin] = getCriticalValuesEventually(phiObj, traj, predicates, timeStamps)
% returns critical time and robustness value for 'Eventually' and 'Globally' operator

if isempty(phiTimeIdx)
    
    tmin = 0;
    dmin = -Inf;
    
else
    
    timeVals = timeStamps(phiTimeIdx)';
    
    dist2Pred = getDistanceToPredicates(traj(phiTimeIdx, :), predicates(predIdx));
    
    if phiNot % if predicate is to be avoided
        dist2Pred = -dist2Pred;
    end
    
    dist2PredPerTime = dist2Pred./timeVals; % measure of how much to move in unit time
%      dist2PredPerTime = dist2Pred; 
    [tmin, dmin] = getCriticalPoint(timeVals, dist2Pred, dist2PredPerTime, phiType, phiSubTimeIdx);    
        
end

end

function [tmin, dmin, Id] = getCriticalPoint(timeVals, distVals, distValsMod, phiType, phiSubTimeIdx)

numSubComps = size(phiSubTimeIdx, 1);

if strcmp(phiType, 'ev')
    
    [~, Id] = max(distValsMod);
    tminVals = timeVals;
    dminVals = distVals;
    
elseif strcmp(phiType, 'alw')
    
    [~, Id] = min(distValsMod);
    tminVals = timeVals;
    dminVals = distVals;
    
elseif strcmp(phiType, 'alw-ev')
    
    for iComp = numSubComps:-1:1
        % all rows/subComps needs to be true (alw : min) --> row is true if any element is positive (ev : max)
        [dminSubComps(iComp), idSub] = max(distValsMod(phiSubTimeIdx(iComp, :)));
        tminVals(iComp) = timeVals(phiSubTimeIdx(iComp, idSub));
        dminVals(iComp) = distVals(phiSubTimeIdx(iComp, idSub));
    end
    
    [~, Id] = min(dminSubComps);
    
elseif strcmp(phiType, 'ev-alw')
    
    for iComp = numSubComps:-1:1
        % any one row/subComp needs to be true (ev : max) --> row is true if all elements are positive (alw : min)
        [dminSubComps(iComp), idSub] = min(distValsMod(phiSubTimeIdx(iComp, :)));
        tminVals(iComp) = timeVals(phiSubTimeIdx(iComp, idSub));
        dminVals(iComp) = distVals(phiSubTimeIdx(iComp, idSub));
    end
    
    [~, Id] = max(dminSubComps);
    
end

tmin = tminVals(Id);
dmin = dminVals(Id);

end