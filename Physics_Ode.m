%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Runge Kutta Method to find x',x'',y' and y''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function output = Physics_Ode(~, xy)
global  ydotdot xdotdot 
global state xyint fG
global Vs V1 LoD Kd deltp lambda delta MgoMp 

%   Constants and conversion factors
% m2k = 1.9438;   %   Conersion factor for m/s to knots
% G = 9.81;       %   Accel due to gravity
% 
% Vs = 36 / m2k;          %   1G stall speed
% V1 = 1.3333 * Vs;       %   V1 Speed
% LoD = 20;               %   L/D
% Kd = 0.08;              %   Derivative gain
% deltp = 0.0225;        %   Parachtue drag parameter
% lambda = G / V1^2;      %   Lambda lift factor
% delta = lambda/LoD;     %   Delta drag factor


%define differential equations
x = xy(1);
xdot = xy(2);
y = xy(3);
ydot = xy(4);

u = [xyint state];

%   Do the calculations for ydotdot and xdotdot
FMA = FMA_Parachute(fG, u, lambda, delta, Kd, MgoMp, deltp);

ydotdot = FMA(1);
xdotdot = FMA(2);

output = [xdot; xdotdot; ydot;  ydotdot];
return
end


