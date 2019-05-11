%% Changeable Parameters
correction = false;
sigma = 5e-5;
angle_threshold = 80;
dt = 0.005;

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

%Other
MPalpha_Px = 0;
MPalpha_Py = 0;
MPalpha_Pz = 0;
MPalpha_Ax = 0;
MPalpha_Ay = 0;
MPalpha_Az = 0;

% sensors = [0.1 0.1 0.0; -0.1 -0.1 0.0; -0.1 0.1 0.0; 0.1 -0.1 0.0; ...
%     0.05 0.05 0.05; -0.05 -0.05 0.05; -0.05 0.05 0.05; 0.05 -0.05 0.05]';
tracker_position = [0.0; 1.0; 1.0];
tracker_angleaxis = [0.0; 0.0; 0.0];
tracker_rotation = eye(3);
% lighthouse_position = [0.0 0.0 0.0]';%; 1.0 0.0 0.0].';
% lighthouse_rotation = [eye(3) roty(270)];
lighthouse_position = [0.0 0.0 0.0]';
lighthouse_rotation = eye(3);

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
counter = 0;
for j = 1:size(lighthouse_position,2)

    % Pose of the lighthouse in the Vive frame
    vRl = lighthouse_rotation(:,(j*3-2):(j*3));
    vPl = lighthouse_position(:,j);
    % Pose of the vive frame relatively to the lighthouse
    lPv = -vRl.' * vPl;
    lRv = vRl.';

    % Position of the sensors in the tracker frame
    % DIM: [N_sensors, 3]
    tPs = sensors;
    tNs = normals;


    for i = 1:size(sensors,2)
        % Frame conversion
        lPs = lRv * (vRt * tPs(:,i) + vPt) + lPv ;
        lNs = lRv * vRt * tNs(:,i);

        s_lPs = subs(lPs,[vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], ...
            [tracker_position' ...
            tracker_angleaxis']);
        s_lNs = subs(lNs,[vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], ...
            [tracker_position' ...
            tracker_angleaxis']);

        if acos(dot(s_lPs/norm(s_lPs), s_lNs)) > angle_threshold * pi/180
           disp([num2str(i) ' out1'])
           continue
        end
        if acos(dot(s_lPs/norm(s_lPs), s_lNs)) < -angle_threshold * pi/180
           disp([num2str(i) ' out2'])
           continue
        end
        disp([num2str(i) ' in'])
   
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
        hAngle = subs(alphaH,[vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], ...
            [tracker_position' ...
            tracker_angleaxis']);
        vAngle = subs(alphaV,[vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], ...
            [tracker_position' ...
            tracker_angleaxis']);
        B = pi/3;
        A = -pi/3;

        % Probability of an horizontal measurement given the parameters
        NalphaH = 1/(sqrt(2*pi))*exp(-0.5 * ((MalphaH - alphaH)/sigma)^2);
%         BalphaH = int(1/(sqrt(2*pi))*exp(-0.5 * ((MalphaH - alphaH)/sigma)^2),MalphaH,0,B);
%         AalphaH = int(1/(sqrt(2*pi))*exp(-0.5 * ((MalphaH - alphaH)/sigma)^2),MalphaH,0,A);
        % Truncated Normal Distribution
        PalphaH = NalphaH / (sigma);% * (BalphaH - AalphaH));
        % Log Likelihood
        LogH = log(subs(PalphaH,MalphaH,hAngle));
        % Probability of an vertical measurement given the parameters
        NalphaV = 1/(sqrt(2*pi))*exp(-0.5 * ((MalphaV - alphaV)/sigma)^2);
%         BalphaV = int(1/(sqrt(2*pi))*exp(-0.5 * ((MalphaV - alphaV)/sigma)^2),MalphaV,0,B);
%         AalphaV = int(1/(sqrt(2*pi))*exp(-0.5 * ((MalphaV - alphaV)/sigma)^2),MalphaV,0,A);
        % Truncated Normal Distribution
        PalphaV = NalphaV / (sigma);% * (BalphaV - AalphaV));
        LogV = log(subs(PalphaV,MalphaV,vAngle));
 
        % Info P_x
        theta = vPt_x;
        MPalpha_Px = MPalpha_Px + subs(diff(diff(LogH,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        MPalpha_Px = MPalpha_Px + subs(diff(diff(LogV,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        % Info P_y
        theta = vPt_y;
        MPalpha_Py = MPalpha_Py + subs(diff(diff(LogH,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        MPalpha_Py = MPalpha_Py + subs(diff(diff(LogV,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);  
        % Info P_z
        theta = vPt_z;
        MPalpha_Pz = MPalpha_Pz + subs(diff(diff(LogH,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        MPalpha_Pz = MPalpha_Pz + subs(diff(diff(LogV,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        % Info A_x
        theta = vAt_x;
        MPalpha_Ax = MPalpha_Ax + subs(diff(diff(LogH,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        MPalpha_Ax = MPalpha_Ax + subs(diff(diff(LogV,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        % Info A_y
        theta = vAt_y;
        MPalpha_Ay = MPalpha_Ay + subs(diff(diff(LogH,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        MPalpha_Ay = MPalpha_Ay + subs(diff(diff(LogV,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        % Info A_z
        theta = vAt_z;
        MPalpha_Az = MPalpha_Az + subs(diff(diff(LogH,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        MPalpha_Az = MPalpha_Az + subs(diff(diff(LogV,theta),theta),...
            [vPt_x vPt_y vPt_z vAt_x vAt_y vAt_z], [tracker_position' tracker_angleaxis']);
        counter = counter + 2;
    end
end

disp(['#Samples ' num2str(counter)]);

Info_Px = -MPalpha_Px;
disp(['V_Px: ' num2str(double(1/Info_Px))]);

Info_Py = -MPalpha_Py;
disp(['V_Py: ' num2str(double(1/Info_Py))]);

Info_Pz = -MPalpha_Pz;
disp(['V_Pz: ' num2str(double(1/Info_Pz))]);

Info_Ax = -MPalpha_Ax;
disp(['V_Ax: ' num2str(double(1/Info_Ax))]);

Info_Ay = -MPalpha_Ay;
disp(['V_Ay: ' num2str(double(1/Info_Ay))]);

Info_Az = -MPalpha_Az;
disp(['V_Az: ' num2str(double(1/Info_Az))]);

return