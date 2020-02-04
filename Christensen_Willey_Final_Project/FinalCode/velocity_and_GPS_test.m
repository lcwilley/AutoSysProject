clear
clc

dt = 0.1;
Ne = 2;
F = [1 0 dt 0 0 0 0 0;
     0 1 0 dt 0 0 0 0;
     0 0 1 0 0 0 0 0;
     0 0 0 1 0 0 0 0;
     0 0 0 0 1 0 dt 0;
     0 0 0 0 0 1 0 dt;
     0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 1];
unitG = [1 0 dt 0;
         0 1 0 dt;
         0 0 1 0;
         0 0 0 1];
% We may initialize G at the beginning, since it's linear and will be
% constant.
G = zeros(4*Ne,4*Ne);
for i = 1:Ne
    G(4*(i-1)+1:4*(i-1)+4,4*(i-1)+1:4*(i-1)+4) = unitG;
end
% It doesn't matter what we initialize sig to be, since sig will be
% initialized by the observation and the associated H matrix.
 
% For this application, there is no control, since we're assuming a = 0
% and velocity is constant. We assume the enemies move without noise,
% which is a fair assumption, since they're ground units
% Therefore, self.sig = G*self.sig*G'.
 
% That takes care of the prediction step
% For the measurement step, I'm assuming I have a range and a bearing to
% the enemy unit.

% Suppose the initial enemy state is [1;1;1;1], such that at time t = 1,
% the enemy will have traveled to the position 2,2. Assume that the initial
% estimate of the state is [1;1;0;0]. After the initializing measurement,
% the enemy state will be [1;1;0;0], since we assumed the enemy to be
% stationary in the first step. Assuming we're located at 0,0 with dt =
% 0.1, we would have a measurement around 1.55 and pi/4. The measurement
% deos not depend on enemy velocity, thus dm/dv = 0. Sigma must be the same
% size as the state, square, so it would be a 4x4. My H is 2x4, since there
% are two measurements and four state variables.
Qe = [0.1 0; 0 0.05];
rx = 0; ry = 0; % UAV position
G = unitG;
% Initialization step
Xe = [1;1;1;1]; % True enemy state
z = [sqrt((Xe(1) - rx).^2 + (Xe(2) - ry).^2);
     wrap_angle(atan2(Xe(2)-ry,Xe(3)-rx))]; % Measurement based on state
Xest = [rx + z(1)*cos(z(2));
        ry + z(2)*sin(z(2));
        0; 0]; % Estimate based on measurement (should be the same without noise)
sig = eye(4);
%  Unpack estimates
ex = Xest(1); ey = Xest(2);
vx = Xest(3); vy = Xest(4);
%  Re-create measurement
del = [ex+vx*dt - rx; ey+vy*dt - ry];
q = del'*del;
% Derivative of measurement with respect to enemy state
H = [del(1)/sqrt(q) del(2)/sqrt(q) del(1)/sqrt(q)*dt del(2)/sqrt(q)*dt;
     -del(1)/q      del(2)/q       -del(1)/q*dt      del(2)/q*dt];
sig = H'/(H*H' + Qe)*H;
% Next time step
Xe1 = Xe;
Xe = G*Xe;
ex = Xe1(1); ey = Xe1(2);
vx = Xe1(3); vy = Xe1(4);
z = [(Xe(1) - rx).^2 + (Xe(2) - ry).^2;
     wrap_angle(atan2(Xe(2)-ry,Xe(3)-rx))];
del = [ex + vx*dt - rx; ey + vy*dt - ry];
q = del'*del;
zhat = [sqrt(q);
        wrap_angle(atan2(del(2),del(1)))];
% Derivative of measurement with respect to enemy state
% H = [d(sqrt(q))/dex d(sqrt(q))/dey d(sqrt(q))/dvx d(sqrt(q))/dvy;
%      d(atan(del))/dex 
% Derivative of atan2 wrt x = -y/(x^2+y^2)
% Derivative of atan2 wrt y = x/(x^2+y^2)
% Derivative of range is x/sqrt(x^2+y^2) and y/sqrt(x^2+y^2)
% Where x and y are the first and second terms respectively for both
% equations, and the chain rule needs to be applied where applicable
H = [del(1)/sqrt(q) del(2)/sqrt(q) del(1)/sqrt(q)*dt del(2)/sqrt(q)*dt;
     -del(1)/q      del(2)/q       -del(1)/q*dt      del(2)/q*dt];
S = H*sig*H' + Qe;
K = sig*H'/S;
Xest = Xest + K*(z-zhat);

% The velocity will never be updated because part of the assumption of the
% model I'm using is constant velocity. I'm going to have to include
% acceleration so that velocity can be updated based on zero acceleration.
% Right? Will that work? I'm still only getting range and bearing
% measurements. Suppose I try it...

%%
dt = .1;
T = 50;%3600*12;
time = 0:dt:T;
xerr = randn()*4.7/3;
yerr = randn()*4.7/3;
err = zeros(2,T/dt+1);
err(:,1) = [xerr; yerr];
for i = 1:length(time)
    xerr = exp(-1/550*dt)*xerr + .21/3*randn();
    yerr = exp(-1/550*dt)*yerr + .21/3*randn();
    err(:,i) = [xerr; yerr];
end
plot(time,err)
disp(std(err(1,:)))
disp(std(err(2,:)))
xlabel('Time (s)')
ylabel('Noise (m)')
legend('Longitude','Latitude')

 
 
 
 