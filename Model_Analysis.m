vp = 50/3;
ve = 125/9;
theta0 = [0,0];
tspan = [0, 15];
alpha = [0, pi/6, pi/4, pi/3, pi/2, pi];
r = [15, 50];
wpMax = vp / 1.5;
weMax = ve / 0.5;
thetap = vp / 1.5;
thetae = ve / 0.5;
dpdt = @(t,p)[vp*cos(wpMax);vp*sin(wpMax)];
dedt = @(t,e)[ve*cos(weMax); ve*sin(weMax)];
opts = odeset('RelTol', 0.0001);

% Model of Thescelosaurus
ue = [0, 0];
[t,E] = ode45(dedt, tspan, ue, opts);
figure
plot(t, E(:,1), 'b-', t, E(:,2), 'r-')
xlabel('t')
legend('Evader x-coordinate', 'Evader y-coordinate')

% Model of Velociraptor(solitary hunting)
% Evader detect predator at 15 meters
up1 = [-1 * r(1) * cos(alpha(1)), r(1) * sin(alpha(1))];
[t,P] = ode45(dpdt, tspan, up1, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Distance between Thescelosaurus and Velociraptor
distance = abs((E(:,2) - P(:,2)) / ((E(:,1) - P(:,1))));
figure
plot(t, distance, 'b-')
xlabel('t')
ylabel('distance')


% Model of Velociraptor(solitary hunting)
% Evader detect predator at 50 meters
up2 = [-1 * r(2) * cos(alpha(1)), r(2) * sin(alpha(1))];
[t,P] = ode45(dpdt, tspan, up2, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Distance between Thescelosaurus and Velociraptor
distance = abs((E(:,2) - P(:,2)) / ((E(:,1) - P(:,1))));
figure
plot(t, distance, 'b-')
xlabel('t')
ylabel('distance')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi/6
up3 = [-1 * r(1) * cos(alpha(2)), r(1) * sin(alpha(2))];
[t,P] = ode45(dpdt, tspan, up3, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 50 meters
% alpha = pi/6
up4 = [-1 * r(2) * cos(alpha(2)), r(2) * sin(alpha(2))];
[t,P] = ode45(dpdt, tspan, up4, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi/4
up5 = [-1 * r(1) * cos(alpha(3)), r(1) * sin(alpha(3))];
[t,P] = ode45(dpdt, tspan, up5, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 50 meters
% alpha = pi/4
up6 = [-1 * r(2) * cos(alpha(3)), r(2) * sin(alpha(3))];
[t,P] = ode45(dpdt, tspan, up6, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi/3
up7 = [-1 * r(1) * cos(alpha(4)), r(1) * sin(alpha(4))];
[t,P] = ode45(dpdt, tspan, up7, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 50 meters
% alpha = pi/3
up8 = [-1 * r(2) * cos(alpha(4)), r(2) * sin(alpha(4))];
[t,P] = ode45(dpdt, tspan, up8, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi/2
up9 = [-1 * r(1) * cos(alpha(5)), r(1) * sin(alpha(5))];
[t,P] = ode45(dpdt, tspan, up9, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 50 meters
% alpha = pi/2
up10 = [-1 * r(2) * cos(alpha(5)), r(2) * sin(alpha(5))];
[t,P] = ode45(dpdt, tspan, up10, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi
up11 = [-1 * r(1) * cos(alpha(6)), r(1) * sin(alpha(6))];
[t,P] = ode45(dpdt, tspan, up11, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi
up12 = [-1 * r(2) * cos(alpha(6)), r(2) * sin(alpha(6))];
[t,P] = ode45(dpdt, tspan, up12, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Pursuer Strategy (Pure Pursuit)
thetaLOS = atan2(dedt(2) - dpdt(2), dedt(1) - dpdt(1));
changeThetap = thetaLOS - thetap; 
wp = wpMax * sign(changeThetap);
dpdt = @(t,p)[vp*cos(wp(1));vp*sin(wp(2))];

% Model of Velociraptor(solitary hunting)
% Evader detect predator at 15 meters
[t,P] = ode45(dpdt, tspan, up1, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Pursuer Strategy(Cooperative Pincer / Lead Pursuit)
% Evader detect predator at 15 meters
ulead = @(t,p)[vp*cos(wpMax);vp*sin(wpMax) + (r(1) * vp*sin(wpMax))];

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi/6
[t,P] = ode45(ulead, tspan, up3, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi/4
[t,P] = ode45(ulead, tspan, up5, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi/3
[t,P] = ode45(ulead, tspan, up7, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi/2
[t,P] = ode45(ulead, tspan, up9, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi
[t,P] = ode45(ulead, tspan, up11, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Pursuer Strategy(Cooperative Pincer / Lead Pursuit)
% Evader detect predator at 15 meters
ulead = @(t,p)[vp*cos(wpMax);vp*sin(wpMax) + (r(2) * vp*sin(wpMax))];

% Model of Velociraptor(pair hunting)
% Evader detect predator at 50 meters
% alpha = pi/6
[t,P] = ode45(ulead, tspan, up4, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 50 meters
% alpha = pi/4
[t,P] = ode45(ulead, tspan, up6, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 50 meters
% alpha = pi/3
[t,P] = ode45(ulead, tspan, up8, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 50 meters
% alpha = pi/2
[t,P] = ode45(ulead, tspan, up10, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(pair hunting)
% Evader detect predator at 15 meters
% alpha = pi
[t,P] = ode45(ulead, tspan, up12, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Evader Strategy (Straight Flight vs. Last-Ditch Jinking Maneuver)

% Model of Thescelosaurus
we = weMax;
for n = 1:t(2)
    dpdt(n) = @(t,p)[vp*cos(wpMax);vp*sin(wpMax)];
    dedt(n) = @(t,e)[ve*cos(we); ve*sin(we)];
    d = dedt(n) - dpdt(n);
    if d > r(1)
        we = 0;
    else
        we = weMax;
    end
end
ue = [0, 0];
[t,E] = ode45(dedt, tspan, ue, opts);
figure
plot(t, E(:,1), 'b-', t, E(:,2), 'r-')
xlabel('t')
legend('Evader x-coordinate', 'Evader y-coordinate')

% Model of Velociraptor(solitary hunting)
% Evader detect predator at 15 meters
up1 = [-1 * r(1) * cos(alpha(1)), r(1) * sin(alpha(1))];
[t,P] = ode45(dpdt, tspan, up1, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Model of Velociraptor(solitary hunting)
% Evader detect predator at 50 meters
up1 = [-1 * r(2) * cos(alpha(1)), r(2) * sin(alpha(1))];
[t,P] = ode45(dpdt, tspan, up1, opts);
figure
plot(t, P(:,1), 'b-', t, P(:,2), 'r-')
xlabel('t')
legend('Pursuer x-coordinate', 'Pursuer y-coordinate')

% Pursuer and Evader Strategy (Bounded Voronoi Cells)