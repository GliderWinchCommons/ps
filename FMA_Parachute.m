function z = FMA_Parachute(fG, u, lambda, delta, Kd, MgoMp, deltp)

persistent vdotold
if isempty(vdotold)
    vdotold = 0;
end


y     = u(3);
x     = u(1);
ydot  = u(4);
xdot  = u(2);
state = u(5);


vsq = xdot^2 + ydot^2;
v = sqrt(vsq);

rsq = x^2 + y^2;
r = sqrt(rsq);

G = 9.81;

fxG = fG * -x / r;
fyG = fG * -y / r;

switch state
    case 0
        ydotdot = 0;
        xdotdot = 0;
        
    case 1  %   Groundroll
        xdotdot = fxG - v * delta * xdot;
        ydotdot = max(fyG + v * (-lambda * xdot - delta * ydot) - G, 0);
    
    case 3  %   Liftoff has occured       
        xdotdot = fxG + v * (lambda * ydot - delta * xdot);
        ydotdot = fyG + v * (-lambda * xdot - delta * ydot) - G;
        
    case 5  %   Airspeed has peaked, damping engaged
        gain = 1 + Kd * vdotold;
        xdotdot = fxG + v * gain * (lambda * ydot - delta * xdot);
        ydotdot = fyG + v * gain * (-lambda * xdot - delta * ydot) - G;
    
    case 7  %   Recovery has commenced
        xdotdot = fxG * MgoMp - v * deltp *  xdot;
        ydotdot = fyG * MgoMp - v * deltp *  ydot - G;  
        
    case 8  %   Should stop soon
        xdotdot = 0;
        ydotdot = 0;
      
    otherwise
        disp('Invalid State');
        display(state);
end

vdotold = (xdot * xdotdot + ydot * ydotdot) / v;

z = [ydotdot xdotdot];




