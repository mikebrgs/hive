%% Params
sigma = 5e-5;

%% Other
%Integration bounds
B = pi/3;
A = -pi/3;

% Tracker pose in vive frame
syms vAt_x vAt_y vAt_z
syms vPt_x vPt_y vPt_z
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

% Pose of the lighthouse in the Vive frame
syms vPl_x vPl_y vPl_z
syms vRl_11 vRl_12 vRl_13 vRl_21 vRl_22 vRl_23 vRl_31 vRl_32 vRl_33
vPl = [vPl_x;vPl_y;vPl_z];
vPl = [0;0;0];
vRl = [vRl_11 vRl_12 vRl_13; vRl_21 vRl_22 vRl_23; vRl_31 vRl_32 vRl_33];
vRl = eye(3);
% Pose of the vive frame relatively to the lighthouse
lPv = -vRl.' * vPl;
lRv = vRl.';

syms tPs_x tPs_y tPs_z
tPs = [0;0;0];

lPs = lRv * (vRt * tPs + vPt) + lPv ;

% probability
x = (lPs(1)/lPs(3));
y = (lPs(2)/lPs(3));
% Symbolic angle
syms alpha
% Horizontal
alphaH = atan(x);
% Vertical
alphaV = atan(y);

theta = vPt_x;
% inner_x_H = (alpha - alphaH)/sigma;
% taylor_exp_H = 1 - inner_x_H.^2/2;% ...
%     + inner_x_H.^4/8;% ...
%     - inner_x_H.^6/48 ...
%     + inner_x_H.^8/384;
NalphaH = 1/(sqrt(2*pi))*exp(-0.5*((alpha - alphaH)/sigma).^2);
InfoH1 = int(diff(diff(NalphaH,theta),theta),alpha,A,B);
disp('InfoH1 Done')
theta = vPt_x;
NalphaV = 1/(sqrt(2*pi))*exp(-0.5*((alpha - alphaV)/sigma).^2);
InfoV1 = int(diff(diff(NalphaV,theta),theta),alpha,A,B);
disp('InfoV1 Done')

% x = linspace(-0.01,0.01,1001)
% y1 = sqrt(2*pi*sigma^2)*exp(-0.5*(x/sigma).^2)
% y2 = sqrt(2*pi*sigma^2)*(1-((x/sigma).^2)/2 + ((x/sigma).^4)/8)