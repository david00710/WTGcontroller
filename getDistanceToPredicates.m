function dist2Pred = getDistanceToPredicates(traj, predicate)

nTime = size(traj, 1); % time is along row; dimension is along column

for iTime = nTime:-1:1
   [dist2Pred(iTime), inSet] = SignedDist(traj(iTime, :)', predicate.A, predicate.b);
end

end

