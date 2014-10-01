function z = State_Parachute(vec)

global state

% if isempty(state)
%     state = 0;
% end


ydotdot = vec(1);
xdotdot = vec(2);
ydot = vec(7);
xdot = vec(5);
y = vec(6);

switch state
    case 0  %   Safe proxy, wait for state message in main program
%         state = 1; 
        
    case 1  %   Ground roll through liftoff
        if ydot > 0 
            state = 3;  
        end
            
    case 3  %   Liftoff has occured
        vsq = xdot^2 + ydot^2;
        v = sqrt(vsq);
        vdot = (xdot * xdotdot + ydot * ydotdot) / v;
        if vdot < 0
            state = 5;
        end
        if y < 0
            state = 8;
        end       
        
    case 5  %   Airpseed has peaked, damping engaged
        if ydot <= 0
            state = 7;            
        end
        if y < 0
            state = 8;
        end
    
    case 7  %   Altitude has peaked, glider released
        if y < 0
            state = 8;
        end
        
    case 8 
        %   Launch should terminate soon
    
    otherwise
        error('Invalid State')
end

z = state;