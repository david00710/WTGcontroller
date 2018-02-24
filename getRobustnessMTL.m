function [nearest_point_on_s, tmin, dmin, umin, i_pr, inSet] = getRobustnessMTL(phiObj, pred, xout, tout, varargin)
% [nearest_point_on_s, tmin, dmin, umin, i_pr, inSet] = getRobustnessMTL(phiObj, pred, xout, tout)
% returns robustness values for each 'phiObj' object passed.


%% Get robustness details for each component (if multiple) of 'phiObj'

for iComp = phiObj.numComps:-1:1
    
    [tminVals(iComp), dminVals(iComp)] = getCriticalValuesMTLpower(phiObj.timeIdx{iComp}, phiObj.subTimeIdx{iComp}, ...
        phiObj.type{iComp}, phiObj.not(iComp), xout, phiObj.predId(iComp), pred, tout, varargin{:});
    predVals(iComp) = phiObj.predId(iComp);
    
end

%% 'phiObj' only has multiple components in case of 'disjunction' in the original MTL spec 'phi' producing 'phiObj'

% if phiObj.numComps > 1 && any(diff(dminVals) == 0)
%    % if different predicates in the disjunction have same robustness, choose the one which needs to be true at a later time
%    [tmin, idx] = max(tminVals);
%    dmin = dminVals(idx);
% else
%    [dmin, idx] = max(dminVals);
%    tmin = tminVals(idx);
% end
tempVal = [];
if phiObj.numComps > 1
    for ii=1: phiObj.st(end)
        st{ii}=find(phiObj.st==ii); 
    [~, id{ii}] = min(dminVals(st{ii})); 
    tempVal = [tempVal dminVals(st{ii}(id{ii}))/tminVals(st{ii}(id{ii}))];
    end

    [~, idx0] = max(tempVal); % choose the one for which needs to move the least unit time
   
    idx =  st{idx0}(id{idx0});
%       idx = 1;
else
    idx = 1;
end

tmin = tminVals(idx);
dmin = dminVals(idx);
nearest_point_on_s = xout(predVals(idx),:)';
i_pr = predVals(idx);

[~, inSet, umin] = SignedDist(nearest_point_on_s, pred(i_pr).A, pred(i_pr).b);

% umin is the projection of the critical point of the trajectory on the
% critical predicate of the MTL specification
end