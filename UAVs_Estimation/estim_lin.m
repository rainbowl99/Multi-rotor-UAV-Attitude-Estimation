function [output]= estim_lin(u)

% Fonction qui d閒init la dynamique de l'observateur lin閍ire pour estimer
% le vecteur des angles de Roulis/Tangage/Lacet (not� "thetahat"). La fonction doit renvoyer dthetahat/dt

thetahat= u(1:3); % Angles estim閟 角度估计
acc= u(4:6); % Mesures acc閘閞om鑤res 加速度计测量
gyro= u(7:9); % Mesures gyrom鑤res 陀螺仪测量
magneto= u(10:12); % Mesures magn閠om鑤res 磁强计测量

g = -9.8;
k1 = 0.55;
k2 = 0.55;
k3 = 0.55;

mmline = magneto - dot(magneto, acc/g) * (acc/g);
mi = 4.7e-5*[cosd(65);0;sind(65)];
e2 = [0, 1, 0];

thetahatdot1 = gyro(1) - k1 * (thetahat(1) + acc(2)/g);
thetahatdot2 = gyro(2) - k2 * (thetahat(2) - acc(1)/g);
thetahatdot3 = gyro(3) - k3 * (thetahat(3) + dot((mmline/mi(1)), e2));

output= [thetahatdot1;thetahatdot2;thetahatdot3];

