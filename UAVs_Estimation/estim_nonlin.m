function [output]= estim_nonlin(u)

% Fonction qui d閒init la dynamique de l'observateur non-lin閍ire (Filtre nonlin閍ire explicit, ou autre) 
% pour estimler le quaternion du rep鑢e corps vers le rep鑢e inertiel (not� "qhat").
% La fonction doit renvoyer dqhat/dt

qhat= u(1:4); % Quaternion estim� 四元数估计
acc= u(5:7); % Mesures accéléromètres 加速度计测量
gyro= u(8:10); % Mesures gyromètres 陀螺仪测量
magneto= u(11:13); % Mesures magnétomètres 磁强计测量

qshat = qhat(1);
qvhat = [qhat(2);qhat(3);qhat(4)];

S_qvhat = [0 -qvhat(3) qvhat(2)
           qvhat(3) 0 -qvhat(1)
           -qvhat(2) qvhat(1) 0] ;

Rhat = eye(3) + 2 * qshat*S_qvhat + 2*S_qvhat^2;

k1 = 0.0075;
k2 = 0.0075;

w_hat = gyro + cross(k1*acc,transpose(Rhat)*[0;0;-9.81]) + cross(k2*magneto,transpose(Rhat)*4.7e-5*[cosd(65); 0 ; sind(65)]);

qs_hat_dot = -0.5 * (dot(qvhat,w_hat) - qshat * (1-norm(qhat)^2));

qv_hat_dot = 0.5 * (qshat * w_hat + cross(qvhat,w_hat) + qvhat * (1-norm(qhat)^2));

output= [qs_hat_dot; qv_hat_dot(1) ;qv_hat_dot(2) ; qv_hat_dot(3)];
