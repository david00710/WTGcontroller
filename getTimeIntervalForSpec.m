function interval = getTimeIntervalForSpec(phi) % timeStamps, timeIdx)
% interval = getTimeIntervalForSpec(phi) returns the time interval for the temporal operator specification 'phi'

% timeStr = phi(regexp(phi, '[\d\.,\d]'));
% sepIndx = regexp(timeStr, ',');
% interval = [str2double(timeStr(1:sepIndx-1)) str2double(timeStr(sepIndx+1:end))];

interval = str2double(regexp(phi, '\d*', 'Match'));

% timeVals = str2double(regexp(phi, '\d*', 'Match'));
% timeValsIdx = round(timeVals/ds)+1;
% timeIdx = timeValsIdx(1):timeValsIdx(2);
% %     timeIdx = timeIdx(timeStamps >= interval(1) & timeStamps <= interval(2));

end

