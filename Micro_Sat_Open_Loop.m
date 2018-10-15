%Advance Control Lab1 - Q7

J_s = 31.38; %Satellite Inertia kg.m^2
w = 2.6268; %Flexible mode freauency rad*s^-1
D_r = 0.00495; %flexible mode damping ratio
a = 0.5736; % coupling coefficient
e = 0.00495; %flexible mode damping ratio
J_r = 0.0004; % reaction wheel inertia kg.m^2
Tc_max = 0.05; % commanded toraue saturation N*m
wr_max = 2800; %reaction wheel velocity saturation rpm
To_m = 0.8; %meqsure,ent delay second
noise = 10^-9; %meqsurement noise variance
T_e = 0.5; %estimator time constant second
T_d = 0.001; % disturbqnce torque N*m
n_0 = 0.0001; %initial condition on the state n

%% Q7 function ss and damp
A = [0 1 0 0; 0 0 -w^2/(1-a) -2*D_r*w/(1-a); 0 0 0 1; 0 0 -w^2/(1-a) -2*D_r*w/(1-a)];
B = [0; 0; 1/((1-a)*J_s); a/((1-a)*J_s)];
C = [1 0 0 0];
D = [0];

sys= ss(A, B, C, D);
damp (sys);

%% Answer of Q7%                                                                        
%          Pole              Damping       Frequency      Time Constant  
%                                        (rad/seconds)      (seconds)    
%                                                                        
%   0.00e+00                -1.00e+00       0.00e+00              Inf    
%   0.00e+00                -1.00e+00       0.00e+00              Inf    
%  -3.05e-02 + 4.02e+00i     7.58e-03       4.02e+00         3.28e+01    
%  -3.05e-02 - 4.02e+00i     7.58e-03       4.02e+00         3.28e+01

%% Q9 linmod 
[A2,B2,C2,D2] = linmod('Satellite_simulink_linmod');
sys = ss(A2,B2,C2,D2);
damp(sys);

%% Q10 undriven response with a nonzero initial condition on the state n
step (sys);
x0= [0; 0; n_0 ; 0];

initial(sys,x0);

%% Q11 display the bode and nichols plots of the system
bode(sys);
nichols(sys);
