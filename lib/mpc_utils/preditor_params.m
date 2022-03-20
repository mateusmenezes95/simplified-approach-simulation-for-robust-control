
% Function to define preditor params for free and forced response within
% prediction and control horizons. This is considered for discrete-time
% systems only.
function [A_pred, B_pred, C_pred, H] = preditor_params(A, B, C, Np, Nu)
   A_pred = [];
   C_pred = [];
   B_pred = [];
   B_pred_collum = [];

   for i=1:Np
   % Computing A_pred
     A_pred= [A_pred;A^i];
   % Computing C_pred
     C_pred_collum = [zeros(size(C,1)*(i-1),size(C,2)) ; C ; zeros(size(C,1)*(Np-i),size(C,2))];
     C_pred = [C_pred C_pred_collum];
					            
   % Computing B_pred_collum;

     B_pred_collum = [B_pred_collum ; A^(i-1)*B];
   end
									    
   for j=1:Nu

     B_pred_collum_temp = [zeros(size(B,1)*(j-1),size(B,2)) ; B_pred_collum];
     B_pred_collum_temp(size(B_pred_collum,1)+1:size(B_pred_collum_temp,1), :) = [];
														            
     B_pred = [B_pred B_pred_collum_temp];
   end

nd
