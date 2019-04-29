syms vRl_11 vRl_12 vRl_13 vRl_21 vRl_22 vRl_23 vRl_31 vRl_32 vRl_33
syms vPl_x vPl_y vPl_z

syms vRt_11 vRt_12 vRt_13 vRt_21 vRt_22 vRt_23 vRt_31 vRt_32 vRt_33
syms vPt_x vPt_y vPt_z

syms lRt_11 lRt_12 lRt_13 lRt_21 lRt_22 lRt_23 lRt_31 lRt_32 lRt_33
syms lPt_x lPt_y lPt_z
syms tPs_x tPs_y tPs_z

vRl = [vRl_11 vRl_12 vRl_13;vRl_21 vRl_22 vRl_23; vRl_31 vRl_32 vRl_33];
vPl = [vPl_x;vPl_y;vPl_z];
lPv = -vRl.' * vPl;
lRv = vRl.';

vRt = [vRt_11 vRt_12 vRt_13;vRt_21 vRt_22 vRt_23; vRt_31 vRt_32 vRt_33];
vPt = [vPt_x;vPt_y;vPt_z];

lRt = [lRt_11 lRt_12 lRt_13;lRt_21 lRt_22 lRt_23; lRt_31 lRt_32 lRt_33];
lPt = [lPt_x;lPt_y;lPt_z];

tPs = [tPs_x;tPs_y;tPs_z];

% Single Lighthouse
% lPs = lRt * tPs + lPt;
lPs = lRv * (vRt * tPs + vPt) + lPv ;

alphaH = atan(lPs(1)/lPs(3));
alphaV = atan(lPs(2)/lPs(3));

syms alpha sigma

PalphaH = 1/(sigma * sqrt(2*pi))*exp(-0.5 * ((alpha - alphaH)/sigma)^2);
logPalphaH = log(PalphaH);

PalphaV = 1/(sigma * sqrt(2*pi))*exp(-0.5 * ((alpha - alphaH)/sigma)^2);
logPalphaV = log(PalphaV);

disp('diff(logPalphaH,vPt)')
ccode(diff(logPalphaH,vPt_x))
ccode(diff(logPalphaH,vPt_y))
ccode(diff(logPalphaH,vPt_z))

disp('diff(logPalphaV,vPt)')
ccode(diff(logPalphaV,vPt_x))
ccode(diff(logPalphaV,vPt_y))
ccode(diff(logPalphaV,vPt_z))