function horizon_refs = get_horizon_references(k, prediction_horizon, future_refs)
% Future references must be in Y = [yr(k) yr(k+1) ... yr(k+N-1)]
  horizon_refs = [];
  for j=k:k+prediction_horizon-1
    if j > length(future_refs)
      horizon_refs = [horizon_refs; future_refs(:,end)];  % Keeps the last reference
    else
      horizon_refs = [horizon_refs; future_refs(:,j)];
    end
  end
end