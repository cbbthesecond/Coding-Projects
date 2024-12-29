%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     Geometry and Grid Calculations                     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                Nozzle                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic;
% Flow Conditions
M_inlet = 0.4;
M_exit = 3.0;

% Constants
gamma = 1.4;
R_air = 287; % J/kg-K

% Assumptions
P_inlet = 2e5;   % Total Pressure (Pa)
T_inlet = 350;   % Total Temperature (K)
convergence_half_angle = 30; % degrees
divergence_half_angle = 15;  % degrees
r_inlet = 0.07;   % meters
A_inlet = pi * r_inlet^2; % m^2

%%% Equations %%%

% Nozzle Radius Sizing Calculations
area_ratio_exit_throat = (1/M_exit) * ((2/(gamma+1)) * (1 + (gamma-1)/2 * M_exit^2))^((gamma+1)/(2*(gamma-1)));
area_ratio_inlet_throat = (1/M_inlet) * ((2/(gamma+1)) * (1 + (gamma-1)/2 * M_inlet^2))^((gamma+1)/(2*(gamma-1)));

A_throat = A_inlet / area_ratio_inlet_throat;
A_exit = A_throat * area_ratio_exit_throat;

r_throat = sqrt(A_throat/pi);
r_exit = sqrt(A_exit/pi);

r_a = 1.5*r_throat;
r_b = 0.45*r_throat;

% Nozzle Length Calculations
L_convergence = (r_inlet - r_throat) / tand(convergence_half_angle);
L_divergence = 3.5*r_throat; % From Rao 1958
L_nozzle = L_convergence + L_divergence;

% Static Properties at Inlet
[T_static_inlet, P_static_inlet, rho_inlet, v_inlet] = calculateStaticProperties(M_inlet, T_inlet, P_inlet, gamma, R_air);

% Mass Flow Rate Calculation
mass_flow_inlet = rho_inlet * A_inlet * v_inlet;

% Atmospheric pressure
P_atm = 101325; % Atmospheric pressure in Pa

% Gauge Pressure Calculation
P_gauge_inlet = P_static_inlet - P_atm;

% **New Section: Throat Static Pressure Calculation**
% ---------------------------------------------------
% Define Mach number at throat
M_throat = 1.0; % Mach number at throat

% Calculate Static Properties at Throat using the same total pressure and temperature
[T_static_throat, P_static_throat, rho_throat, v_throat] = calculateStaticProperties(M_throat, T_inlet, P_inlet, gamma, R_air);

% Display Results
fprintf('Nozzle Parameters\n-------------------\n');
fprintf('Throat Radius: %.4f m\n', r_throat);
fprintf('R_a: %.4f m\n', r_a);
fprintf('R_b: %.4f m\n', r_b);
fprintf('Exit Radius: %.4f m\n', r_exit);
fprintf('Converging Length: %.4f m\n', L_convergence);
fprintf('Diverging Length: %.4f m\n', L_divergence);
fprintf('Total Nozzle Length: %.4f m\n', L_nozzle);
fprintf('Mass Flow Rate: %.4f kg/s\n', mass_flow_inlet);
fprintf('Inlet Gauge Pressure: %.4f Pa\n', P_gauge_inlet);
fprintf('Throat Static Pressure: %.4f Pa\n\n', P_static_throat); % **New Line**
% ---------------------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                Grid                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculations for y+ calculator
reference_Length = L_nozzle;
desired_y_plus = 1;

% Dynamic Viscosity using Sutherland's Law
mu_ref = 1.716e-5; % kg/(m·s) at T_ref
T_ref = 273; % K
suth_constant_air = 111; % K
dynamic_viscosity_inlet = mu_ref * ((T_static_inlet / T_ref)^(3/2)) * (T_ref + suth_constant_air) / (T_static_inlet + suth_constant_air);

% Density at inlet
density_inlet = rho_inlet;

fprintf('Wall Spacing Calculator Parameters\n-----------------------------------\n');
fprintf('Inlet Velocity: %.4f m/s\n', v_inlet);
fprintf('Reference Length: %.4f m\n', reference_Length);
fprintf('Desired y+: %.2f\n', desired_y_plus);
fprintf('Dynamic Viscosity: %.10f kg/(m·s)\n', dynamic_viscosity_inlet);
fprintf('Density: %.4f kg/m^3\n\n', density_inlet);

toc;
