%% Create data
x = [1 2 3 4 5 6]';
t = (0:0.02:2*pi)';
A = [sin(t) sin(2*t) sin(3*t) sin(4*t) sin(5*t) sin(6*t)];
e = (-4+8*rand(length(t),1));
e(100:115) = 30;
y = A*x+e;

%% Various estimates
xhat = sdpvar(6,1);
e = y-A*xhat;

bound = sdpvar(length(e),1);
Constraints = [-bound <= e <= bound];
optimize(Constraints,sum(bound));
x_L1 = value(xhat);

optimize([],e'*e);
x_L2 = value(xhat);

bound = sdpvar(1,1);
Constraints  = [-bound <= e <= bound];

optimize(Constraints,bound);
x_Linf = value(xhat);
         
bound = sdpvar(length(e),1);
Constraints = [-bound <= e <= bound];         
optimize(Constraints,e'*e + sum(bound));
x_L2L1 = value(xhat);

%% Evaluate
plot(t,[y A*x_L1 A*x_L2 A*x_Linf A*x_L2L1]);
legend('y','L1','L2','Linf','L2/L1')