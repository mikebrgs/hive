%% Changeable Parameters
correction = false;
sigma = 5e-5;

%% Script
format long;
% Pose of the tracker in the vive frame
syms vAt_x vAt_y vAt_z
syms vPt_x vPt_y vPt_z
% Correction parameters
phase = 0.0;
tilt = 0.0;
gib_phase = 0.0;
gib_mag = 0.0;
curve = 0.0;

% Results
final_bound_Px = 0;
final_bound_Py = 0;
final_bound_Pz = 0;
final_bound_Ax = 0;
final_bound_Ay = 0;
final_bound_Az = 0;
%Other
variables = [];
MPalphaH = 1;
MPalphaV = 1;

sensors = [0.1 0.1 0.0; -0.1 -0.1 0.0; -0.1 0.1 0.0; 0.1 -0.1 0.0; ...
    0.05 0.05 0.05; -0.05 -0.05 0.05; -0.05 0.05 0.05; 0.05 -0.05 0.05]';
tracker_position = [0.0; 0.0; 1.0];
tracker_angleaxis = [0.0; 0.0; 0.0];
tracker_rotation = eye(3);
lighthouse_position = [0.0 0.0; 0.0 0.0; 0.0 2.0];
lighthouse_rotation = [eye(3) roty(180)];

% Pose of the tracker in the Vive frame
vAt = [vAt_x vAt_y vAt_z].';
vOMEGAt = [0.0 -vAt_z vAt_y; vAt_z 0.0 -vAt_x; -vAt_y vAt_x 0.0];
vRt = eye(3) + vOMEGAt + 1/factorial(2) * vOMEGAt^2 + ...
    1/factorial(3) * vOMEGAt^3 + ...
    1/factorial(4) * vOMEGAt^4 + ...
    1/factorial(5) * vOMEGAt^5 + ...
    1/factorial(6) * vOMEGAt^6 + ...
    1/factorial(7) * vOMEGAt^7 + ...
    1/factorial(8) * vOMEGAt^8 + ...
    1/factorial(9) * vOMEGAt^9;
vPt = [vPt_x;vPt_y;vPt_z];

for j = 1:1

    % Pose of the lighthouse in the Vive frame
    vRl = lighthouse_rotation(:,(j*3-2):(j*3));
    vPl = lighthouse_position(:,j);
    % Pose of the vive frame relatively to the lighthouse
    lPv = -vRl.' * vPl;
    lRv = vRl.';

    % Position of the sensors in the tracker frame
    % DIM: [N_sensors, 3]
    tPs = sensors;


    for i = 1:1
        % Frame conversion
        lPs = lRv * (vRt * tPs(:,i) + vPt) + lPv ;

        x = (lPs(1)/lPs(3));
        y = (lPs(2)/lPs(3));

        if (correction)
            % Horizontal
            alphaH = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
            % Vertical
            alphaV = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;
        else
            % Horizontal
            alphaH = atan(x);
            % Vertical
            alphaV = atan(y);
        end

        % Measurement and measurement noise
        horizontal_string = ['MalphaH' num2str(i) '_' num2str(j)];
        vertical_string = ['MalphaV' num2str(i) '_' num2str(j)];
        MalphaH = sym(horizontal_string);
        MalphaV = sym(vertical_string);
        variables = [variables, MalphaH];
        variables = [variables, MalphaV];
        B = pi/3;
        A = -pi/3;

        % Probability of an horizontal measurement given the parameters
        NalphaH = 1/(sqrt(2*pi))*exp(-0.5 * ((MalphaH - alphaH)/sigma)^2);
        BalphaH = int(1/(sqrt(2*pi))*exp(-0.5 * ((MalphaH - alphaH)/sigma)^2),MalphaH,0,B);
        AalphaH = int(1/(sqrt(2*pi))*exp(-0.5 * ((MalphaH - alphaH)/sigma)^2),MalphaH,0,A);
        % Truncated Normal Distribution
        PalphaH = NalphaH / (sigma);% * (BalphaH - AalphaH));
        % log likedlihood
        MPalphaH = MPalphaH * PalphaH;

        % Probability of an vertical measurement given the parameters
        NalphaV = 1/(sqrt(2*pi))*exp(-0.5 * ((MalphaV - alphaV)/sigma)^2);
        BalphaV = int(1/(sqrt(2*pi))*exp(-0.5 * ((MalphaV - alphaV)/sigma)^2),MalphaV,0,B);
        AalphaV = int(1/(sqrt(2*pi))*exp(-0.5 * ((MalphaV - alphaV)/sigma)^2),MalphaV,0,A);
        % Truncated Normal Distribution
        PalphaV = NalphaV / (sigma);% * (BalphaV - AalphaV));
        % log likedlihood
        MPalphaV = MPalphaV * PalphaV;


    end
end

MPalpha = MPalphaH * MPalphaV;

return