clc
clear all
A=[0.8218 0 0 
   0 0.8218 0
   0 0      0.5888];

B=[0 0.04 -0.04;
   -0.0461 0.0231 0.0231;
   0.9868 0.9868 0.9868];
C=eye(3);

Np=10;
Nu=10;
r=1;
q=1;


[A_pred, B_pred, C_pred] = preditor_params(A, B, C, Np, Nu);


R = eye(Nu*size(B,2))*r;
Q = eye(Np*size(A,1))*q;
H = (B_pred'*C_pred'*Q*C_pred*B_pred+R)*2;
		          
f_QCB = Q*C_pred*B_pred;
f_AC  = A_pred'*C_pred';

% ---------------------------------Fim do script de inicializacao

x_atual=[0 0 0]';
y_r=[1 1 0]';


W_ref = repmat(y_r, [Np,1]);

f=(2*(x_atual'*f_AC-W_ref')*f_QCB)';

u_vetor=quadprog(H,f);


