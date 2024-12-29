function FinalProject_Corrected()
    clc; clear;

    % Start a parallel pool if not already started
    poolobj = gcp('nocreate');
    if isempty(poolobj)
        parpool;
    end

    tic;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                Nozzle                                   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Constants
    gamma = 1.4;
    R_air = 287; % J/kg-K

    % Assumptions
    P_inlet = 2e5;   % Total Pressure (Pa)
    p_atm = 0;
    T_total_inlet = 350;  % Total Temperature (K)
    r_inlet = 0.07;   % meters
    M_inlet = 0.6;          % Inlet Mach Number

    % Static Properties at Inlet
    [~, P_static_inlet, ~, ~] = calculateStaticProperties(M_inlet, T_total_inlet, P_inlet, gamma, R_air);

    % Calculate Total Pressure at Inlet using isentropic relations
    P_total_inlet = P_static_inlet * (1 + ((gamma - 1)/2) * M_inlet^2)^(gamma / (gamma - 1));

    % Calculate Static Temperature at Inlet using isentropic relations
    T_static_inlet = T_total_inlet / (1 + ((gamma - 1)/2) * M_inlet^2);

    % Calculate Density at Inlet using Ideal Gas Law
    rho_inlet = P_static_inlet / (R_air * T_static_inlet);

    % Calculate Inlet Gauge Pressure
    P_gauge_inlet = P_static_inlet - p_atm;
    fprintf('Inlet Gauge Pressure: %.2f Pa\n', P_gauge_inlet);

    % File path to the STEP file
    stepFilePath = 'CFD_Final_Project_Nozzle1_1_Flow.STEP'; % Update path as needed

    % Import the geometry from the STEP file
    geometry = importGeometry(stepFilePath);

    % Display the imported 3D geometry for verification
    figure;
    pdegplot(geometry, 'FaceAlpha', 0.5);
    title('Imported 3D Nozzle Geometry');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %           Generate Mesh and Extract 2D Cross-Section                    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Create a PDE model for meshing
    model = createpde();
    model.Geometry = geometry;

    % Generate a mesh with finer resolution
    mesh = generateMesh(model, 'GeometricOrder', 'linear');
    % Extract nodes from the mesh
    nodes = mesh.Nodes; % Node coordinates [X; Y; Z]

    % Identify nodes close to the specified X plane (axisymmetric profile)
    x_plane = -2.73932; % Adjust based on geometry
    x_threshold = 1e-3;
    cross_section_indices = abs(nodes(1, :) - x_plane) < x_threshold & (nodes(3, :) >= 0);

    % Extract Y and Z coordinates of the cross-section
    y_cross_section = nodes(2, cross_section_indices); % Y-coordinates (axial distance)
    z_cross_section = nodes(3, cross_section_indices); % Z-coordinates (radius)

    % Visualization 1: Raw Cross-Section
    figure;
    scatter(y_cross_section, z_cross_section, 36, 'filled');
    title('Raw Extracted Cross-Section');
    xlabel('Axial Distance (Y)');
    ylabel('Radius (Z)');
    grid on;

    % Check if cross-section nodes are found
    if isempty(y_cross_section)
        error('No nodes found on the specified X plane. Adjust x_plane or x_threshold or check geometry.');
    end

    % Remove duplicate or noisy points
    [y_unique, unique_indices] = unique(y_cross_section);
    z_filtered = z_cross_section(unique_indices);

    % Sort the points by Y (axial distance)
    [y_sorted, sort_indices] = sort(y_unique);
    z_sorted = z_filtered(sort_indices);

    % Ensure y_sorted and z_sorted are column vectors
    y_sorted = y_sorted(:)./10; % Convert to meters if input is in mm
    z_sorted = z_sorted(:)./10;

    % Adjust y_sorted to start from zero at the inlet
    y_positions_unique = y_sorted - min(y_sorted);

    % Visualization 2: Processed Cross-Section
    figure;
    scatter(y_positions_unique, z_sorted, 36, 'filled');
    title('Processed Cross-Section');
    xlabel('Axial Distance from Inlet (m)');
    ylabel('Radius (Z, m)');
    grid on;
    axis equal;

    % Interpolate if points are too few
    num_points = length(y_positions_unique);
    if num_points < 50
        y_new = linspace(min(y_positions_unique), max(y_positions_unique), 1000);
        z_new = interp1(y_positions_unique, z_sorted, y_new, 'spline');
        valid = ~isnan(z_new);
        y_positions_unique = y_new(valid);
        z_sorted = z_new(valid);

        % Visualization 3: Interpolated Cross-Section
        figure;
        plot(y_positions_unique, z_sorted, 'b-', 'LineWidth', 2);
        title('Interpolated Cross-Section');
        xlabel('Axial Distance from Inlet (m)');
        ylabel('Radius (Z, m)');
        grid on;
        axis equal;
    end

    % Reverse arrays to have inlet at index 1
    radii_unique = flip(z_sorted);
    areas = pi * radii_unique.^2;

    % Identify throat area
    [A_throat, throat_index] = min(areas);
    A_inlet = areas(1);
    A_exit = areas(end);

    % Calculate target location for the shock from the inlet
    distance_from_outlet = 0.076206; % Distance of shock from outlet in meters
    target_location = max(y_positions_unique) - distance_from_outlet; % Location of shock from inlet

    % Identify the index of the point closest to the target location
    [~, target_index] = min(abs(y_positions_unique - target_location));

    % Calculate shock radius at the target location
    shock_radius = radii_unique(target_index);

    % Visualization: Nozzle Profile with Shock Location
    figure;
    hold on;
    plot(y_positions_unique, radii_unique, 'b-', 'LineWidth', 2); % Nozzle profile
    plot([min(y_positions_unique), max(y_positions_unique)], [0, 0], 'k--', 'LineWidth', 1); % Nozzle axis
    plot([y_positions_unique(target_index), y_positions_unique(target_index)], [0, shock_radius], 'r-', 'LineWidth', 2); % Shock location
    legend('Nozzle Profile', 'Nozzle Axis', 'Shock Location', 'Location', 'best');
    xlabel('Axial Distance from Inlet (m)');
    ylabel('Radius (Z, m)');
    title('Nozzle Profile with Shock Location');
    grid on;
    axis equal;
    hold off;

    % Display Debug Information
    fprintf('\nDebug Information:\n');
    fprintf('Number of Nodes in Extracted Cross-Section: %d\n', length(y_cross_section));
    fprintf('Number of Unique Points After Filtering: %d\n', length(y_positions_unique));
    fprintf('Throat Radius: %.4f m, Throat Area: %.4f m^2\n', radii_unique(throat_index), A_throat);
    fprintf('Inlet Radius: %.4f m, Inlet Area: %.4f m^2\n', radii_unique(1), A_inlet);
    fprintf('Exit Radius: %.4f m, Exit Area: %.4f m^2\n', radii_unique(end), A_exit);

    % Optional: Check monotonicity of y_positions_unique
    assert(all(diff(y_positions_unique) > 0), 'y_positions_unique is not monotonically increasing!');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     Calculate Design Exit Pressure (P_exit)                         %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Calculate area ratio at the exit
    area_ratio_exit = A_exit / A_throat;

    % Define the area-Mach number relation function
    area_mach_function = @(M) nozzleAreaMachRelation(M, gamma) - area_ratio_exit;

    % Solve for M_exit numerically
    M_exit_guess = 2.0; % Initial guess for supersonic exit Mach number
    options = optimoptions('fsolve', 'Display', 'none');
    M_exit = fsolve(area_mach_function, M_exit_guess, options);

    % Calculate P_exit using isentropic relations
    P_exit = P_total_inlet / (1 + ((gamma -1) / 2) * M_exit^2)^(gamma / (gamma -1));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     Calculate Back Pressure for Perfect Expansion                    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    P_back_perfect = P_exit;
    fprintf('Back Pressure for Perfect Expansion: %.2f Pa\n', P_back_perfect);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     Calculate Mass Flow Rate at Inlet                                  %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Calculate flow velocity at inlet
    a_inlet = sqrt(gamma * R_air * T_static_inlet);
    V_inlet = M_inlet * a_inlet;

    % Calculate mass flow rate at inlet
    mass_flow_rate = rho_inlet * A_inlet * V_inlet;

    fprintf('Mass Flow Rate at Inlet: %.4f kg/s\n', mass_flow_rate);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     Calculate Back Pressure to Cause Shock at 0.076206m from Outlet     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    target_location = max(y_positions_unique) - 0.076206; % Location of shock from inlet (m)
    [~, target_index] = min(abs(y_positions_unique - target_location));
    
    % Calculate area ratio at the shock location
    area_ratio_shock = areas(target_index) / A_throat;
    
    % Solve for Mach number at shock location (supersonic)
    M_target = nozzleAreaMachRelationInverse(area_ratio_shock, gamma);
    fprintf('Mach number at shock location: %.4f\n', M_target);
    
    % Compute static pressure and temperature at the shock location using isentropic relations
    P_static_target = P_total_inlet / (1 + ((gamma -1)/2) * M_target^2)^(gamma / (gamma -1));
    T_static_target = T_total_inlet / (1 + ((gamma -1)/2) * M_target^2);
    fprintf('Static Pressure at Shock Location: %.2f Pa\n', P_static_target);
    fprintf('Static Temperature at Shock Location: %.2f K\n', T_static_target);
    
    % Apply normal shock relations using static pressure and temperature at shock location
    [M2_target, P2_target, ~] = normalShockRelations(M_target, P_static_target, T_static_target, gamma);
    
    P_back_shock_location = P2_target;
    
    % Calculate shock thickness
    dynamic_viscosity = 1.716e-5 * (T_static_target / 273)^(3/2) * (273 + 111) / (T_static_target + 111); % Sutherland's Law
    shock_thickness = 2 * dynamic_viscosity / V_inlet;
    fprintf('Shock Thickness at Shock Location: %.6e m\n', shock_thickness);
    
    fprintf('Back Pressure to cause a shock at %.4f m from outlet: %.2f Pa\n', 0.076206, P_back_shock_location);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %         Visualization of Nozzle Profile with Shock Location            %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    figure;
    hold on;

    % Nozzle profile from inlet to exit in Y-Z plane
    plot(y_positions_unique, radii_unique, 'b-', 'LineWidth', 2);

    % Add a horizontal line representing the nozzle axis (Z=0)
    y_min = min(y_positions_unique);
    y_max = max(y_positions_unique);
    plot([y_min, y_max], [0, 0], 'k--', 'LineWidth', 1);

    % Plot shock location
    shock_radius = radii_unique(target_index);
    plot([y_positions_unique(target_index), y_positions_unique(target_index)], [0, shock_radius], 'r-', 'LineWidth', 2);
    legend('Nozzle Profile', 'Nozzle Axis', 'Shock Location', 'Location', 'best');

    xlabel('Axial Distance from Inlet (m)');
    ylabel('Radius (Z, m)');
    title('Nozzle Profile with Shock Location at 0.076206 m from Outlet');
    grid on;
    axis equal;
    hold off;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %               Plot Post-Shock Temperature vs Back Pressure             %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Display results
    fprintf('\nSimulation Setup Complete.\n');
    fprintf('Set the outlet gauge pressure in ANSYS Fluent to %.2f Pa for perfect expansion.\n', P_back_perfect);
    fprintf('Set the outlet gauge pressure in ANSYS Fluent to %.2f Pa to achieve a shock at %.4f m from outlet.\n', P_back_shock_location, 0.076206);

    toc
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Subfunctions                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function val = nozzleAreaMachRelation(M, gamma)
    % Area-Mach number relation for a nozzle
    val = (1 ./ M) .* ((2 / (gamma +1)) .* (1 + (gamma -1)/2 .* M.^2)).^((gamma +1)/(2*(gamma -1)));
end

function M = nozzleAreaMachRelationInverse(area_ratio, gamma)
    % Inverse of the area-Mach number relation to find supersonic Mach number from area ratio
    fun = @(M) nozzleAreaMachRelation(M, gamma) - area_ratio;
    M_guess = 2.0; % Initial guess for supersonic Mach number
    options = optimoptions('fsolve', 'Display', 'none');
    M = fsolve(fun, M_guess, options);
    if M < 1
        error('No supersonic solution found for the given area ratio.');
    end
end

function [Temperature, Pressure, Density] = computeFlowProperties(Mach, T_total, P_total, gamma, R_air)
    % Isentropic flow properties
    Temperature = T_total ./ (1 + (gamma - 1)/2 .* Mach.^2);
    Pressure = P_total ./ (1 + (gamma -1)/2 .* Mach.^2).^(gamma / (gamma -1));
    Density = Pressure ./ (R_air .* Temperature);
end

function [M2, P2, T2] = normalShockRelations(M1, P1, T1, gamma)
    % Normal shock relations for downstream Mach number, pressure, and temperature
    M2 = sqrt(((gamma -1)*M1^2 + 2)/(2*gamma*M1^2 - (gamma -1)));
    P2 = P1 * ((2*gamma*M1^2 - (gamma -1))/(gamma +1));
    T2 = T1 * (((2*gamma*M1^2 - (gamma -1)) * ((gamma -1)*M1^2 + 2)) / ((gamma +1)^2 * M1^2));
end
