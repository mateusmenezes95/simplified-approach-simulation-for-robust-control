function [A_pred, B_pred, C_pred] = preditor_params(A, B, C, Np, Nu)
% Function to define preditor params for free and forced response within
% prediction (Np) and control horizons (control_horizon). This is considered for discrete-time
% systems only.
  A_pred = [];
  C_pred = [];
  B_pred = [];
  B_pred_column = [];

  for i=1:Np
    % Computing A_pred
    A_pred= [A_pred;A^i];

    % Computing C_pred
    C_pred_column = [zeros(size(C,1)*(i-1),size(C,2)) ; C ; zeros(size(C,1)*(Np-i),size(C,2))];
    C_pred = [C_pred C_pred_column];

    % Computing B_pred_column;
    B_pred_column = [B_pred_column ; A^(i-1)*B];
  end

  for j=1:Nu
    B_pred_column_temp = [zeros(size(B,1)*(j-1),size(B,2)) ; B_pred_column];
    B_pred_column_temp(size(B_pred_column,1)+1:size(B_pred_column_temp,1), :) = [];
    B_pred = [B_pred B_pred_column_temp];
  end
end
