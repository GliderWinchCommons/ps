function [] = pltrslts(data, i, Mg, Tdsp)
%   Plot results

m2f = 3.281;    %   Converstion factor for meters to feet
m2k = 1.9438;   %   Conersion factor for m/s to knots
G = 9.81;       %   Accel due to gravity

tmpdata = data(1 : i, :);    %   extract data
fGc = tmpdata(:,3);
x = tmpdata(:,4);
y = tmpdata(:,6);
xdot = tmpdata(:,5);
ydot = tmpdata(:,7);
t = tmpdata(:,8);

vsqr = xdot.^2 + ydot.^2;
v = sqrt(vsqr);
theta = atan2(y, x);
thetadeg = theta * 180/pi;
rsq = x.^2 + y.^2;
r = sqrt(rsq);
rdot = -(x .* xdot + y .* ydot) ./ r;


%   Rescale for presentation

v = m2k * v;        %   convert to knots
rdot = m2k * rdot;  %   convert to knots
f = fGc/G;          %   Tension factor

%   Plot the results
figure(1)
%  Plot Time Trajectory
subplot(2, 1, 1)
plot(t, rdot * 10, t, v * 10, t, f * Mg, t, y, ...
    t, 10 * thetadeg, 'linewidth', 2)
axis([0 Tdsp 0  900])
xlabel('Time (seconds)')
grid on

%   plot the xy trajectory
subplot(2, 1, 2)
plot(x, y, 'linewidth', 2)
lmts = axis;
axis([0 lmts(2) 0 0.5 * lmts(2)])
xlabel('x (m)')
ylabel('y (m)')
grid on

end

