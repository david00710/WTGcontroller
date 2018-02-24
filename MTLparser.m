function phiObj = MTLparser(phi, timeStamps, ds)
% phiObj = MTLparser(phi, timeStamps)
% returns 'phiObj' object by parsing the input MTL spec 'phi'
% if 'phi' has disjunctions, then each part of disjunction is considered to be a 'component phi' of 'phiObj'

predNames = 'a':'z';
predIdx = 1:26;
timeIdxAll = 1:numel(timeStamps);

phi=strrep(phi,'(','');
phi=strrep(phi,')','');
disjuncIndx = regexp(phi, '\\/');
conjuncIndx = regexp(phi, '/\');

juncIndx = sort([disjuncIndx conjuncIndx]);

numComps = numel(disjuncIndx) + numel(conjuncIndx)+ 1;
phiObj.numComps = numComps;

horizonLen = round(timeStamps(end)/ds)+1;

ii=1;
if numComps == 1
   phiObj.phi{1} = phi;
   phiObj.st(1) = ii;
else

    phiObj.phi{1} = strtrim(phi(1:juncIndx(1)-1));
    phiObj.st(1) = ii;
        if phi(juncIndx(1)) == '\'
            ii = ii+1;
        end
    for iComp = 2:numComps-1
        phiObj.phi{iComp} = strtrim(phi(juncIndx(iComp-1)+2:juncIndx(iComp)-1));
        phiObj.st(iComp) = ii;
        if phi(juncIndx(iComp)) == '\'
            ii = ii+1;
        end
    end
    
    phiObj.phi{numComps} = strtrim(phi(juncIndx(numComps-1)+2:end));
    phiObj.st(numComps) = ii;
end


for iComp = 1:numComps

    glblIndx = regexp(phiObj.phi{iComp}, 'G');
    evIndx = regexp(phiObj.phi{iComp}, 'F');
    
    interval = getTimeIntervalForSpec(phiObj.phi{iComp});
    phiObj.interval{iComp} = interval;
    
    if ~isempty(glblIndx) && ~isempty(evIndx)
        
%         timeIdxSubComps = round(interval(1))/ds+1:round(interval(2))/ds+1;
%         timeIdxSubCompInterVals = (round(interval(3))/ds+1:round(interval(4))/ds+1)-1;
          timeIdxSubComps = round(interval(1))/ds+1:round(interval(2))/ds;
          timeIdxSubCompInterVals = (round(interval(3))/ds+1:round(interval(4))/ds)-1;
      
        maxSubHorizonLen = timeIdxSubComps + timeIdxSubCompInterVals(end);
        maxSubHorizonLen(maxSubHorizonLen > horizonLen) = [];
        
        numSubComps = numel(maxSubHorizonLen);
        numSubCompsLength = numel(timeIdxSubCompInterVals);
        
        for iSubComp = numSubComps:-1:1
            timeIdx(iSubComp, 1:numSubCompsLength) = timeIdxSubComps(iSubComp) + timeIdxSubCompInterVals;
            subTimeIdx(iSubComp, 1:numSubCompsLength) = iSubComp + timeIdxSubCompInterVals;
        end
        
        timeIdxUniq = unique(timeIdx);
        %             timeIdxUniqTmp = 1:numel(timeIdxUniq);
        
        %             for iSubComp = numSubComps:-1:1
        %                for iLen = numSubCompsLength:-1:1
        %                    subTimeIdx(iSubComp, iLen) = timeIdxUniqTmp(timeIdxUniq == timeIdx(iSubComp, iLen));
        %                end
        %             end
        
        phiObj.timeIdx{iComp} = timeIdxUniq;
        phiObj.subTimeIdx{iComp} = subTimeIdx;
        
        if glblIndx > evIndx
            phiObj.type{iComp} = 'ev-alw'; % globally appears after eventually : <>_[a,b] []_[c,d] p
        else
            phiObj.type{iComp} = 'alw-ev';
        end
        
        clear timeIdx subTimeIdx
    else
        
        phiObj.timeIdx{iComp} = timeIdxAll(timeStamps >= interval(1) & timeStamps <= interval(2));
        phiObj.subTimeIdx{iComp} = [];
        
        if ~isempty(glblIndx)
            phiObj.type{iComp} = 'alw';
        elseif ~isempty(evIndx)
            phiObj.type{iComp} = 'ev';
        end
        
    end
    
    phiObj.predId(iComp) = predIdx(predNames == phiObj.phi{iComp}(end));
    
    if phiObj.phi{iComp}(end-1) == '!'
        phiObj.not(iComp) = 1;
    else
        phiObj.not(iComp) = 0;
    end
    
end

% end

end

