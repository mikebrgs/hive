syms vQt_w vQt_x vQt_y vQt_z vPt_x vPt_y vPt_z tlPs_x tlPs_y tlPs_z tW_x tW_y tW_z tA_x tA_y tA_z vPl_x vPl_y vPl_z vRl_11 vRl_12 vRl_13 vRl_21 vRl_22 vRl_23 vRl_31 vRl_32 vRl_33
syms vG_x vG_y vG_z tB_x tB_y tB_z
syms tRtl_11 tRtl_12 tRtl_13 tRtl_21 tRtl_22 tRtl_23 tRtl_31 tRtl_32 tRtl_33 tPtl_x tPtl_y tPtl_z

% Auxiliary steps
tx  = 2*vQt_x;
ty  = 2*vQt_y;
tz  = 2*vQt_z;
twx = tx*vQt_w;
twy = ty*vQt_w;
twz = tz*vQt_w;
txx = tx*vQt_x;
txy = ty*vQt_x;
txz = tz*vQt_x;
tyy = ty*vQt_y;
tyz = tz*vQt_y;
tzz = tz*vQt_z;
r11 = 1-(tyy+tzz);
r12 = txy-twz;
r13 = txz+twy;
r21 = txy+twz;
r22 = 1-(txx+tzz);
r23 = tyz-twx;
r31 = txz-twy;
r32 = tyz+twx;
r33 = 1-(txx+tyy);

vRt = [r11 r12 r13;r21 r22 r23; r31 r32 r33];
vPt = [vPt_x;vPt_y;vPt_z];
tlPs = [tlPs_x;tlPs_y;tlPs_z];
vPl = [vPl_x;vPl_y;vPl_z];
vRl = [vRl_11 vRl_12 vRl_13;vRl_21 vRl_22 vRl_23;vRl_31 vRl_32 vRl_33];

% tracker's light frame to imu tracker's imu frame
tRtl = [tRtl_11 tRtl_12 tRtl_13;tRtl_21 tRtl_22 tRtl_23;tRtl_31 tRtl_32 tRtl_33];
tPtl = [tPtl_x;tPtl_y;tPtl_z];


Omega = [-vQt_x -vQt_y -vQt_z; vQt_w -vQt_z vQt_y; vQt_z vQt_w -vQt_x; -vQt_y vQt_x vQt_w];
tW = [tW_x;tW_y;tW_z];
tA = [tA_x;tA_y;tA_z];
tB = [tB_x;tB_y;tB_z];
vG = [vG_x;vG_y;vG_z];

lRv = vRl.';
lPv = - lRv * vPl;

lRt = lRv * vRt;
lPt = lRv * vPt + lPv;

lPs = lRt * (tRtl * tlPs + tPtl) + lPt;

% Check if vG or -vG
disp('d dot(V) / dq')
ccode(diff(vRt * tA + vG,vQt_w))
ccode(diff(vRt * tA + vG,vQt_x))
ccode(diff(vRt * tA + vG,vQt_y))
ccode(diff(vRt * tA + vG,vQt_z))

disp('d dot(Q) / dQ')
ccode(diff(0.5 * Omega * (tW-tB),vQt_w))
ccode(diff(0.5 * Omega * (tW-tB),vQt_x))
ccode(diff(0.5 * Omega * (tW-tB),vQt_y))
ccode(diff(0.5 * Omega * (tW-tB),vQt_z))

x = (lPs(1)/lPs(3));
y = (lPs(2)/lPs(3));
syms phase tilt gib_phase gib_mag curve
alphaH = atan(x) - phase - tan(tilt) * y - curve * y * y - sin(gib_phase + atan(x)) * gib_mag;
alphaV = atan(y) - phase - tan(tilt) * x - curve * x * x - sin(gib_phase + atan(y)) * gib_mag;

disp('d alphaH / dP')
ccode(diff(alphaH, vPt_x))
ccode(diff(alphaH, vPt_y))
ccode(diff(alphaH, vPt_z))
disp('d alphaH / dQ')
ccode(diff(alphaH, vQt_w))
ccode(diff(alphaH, vQt_x))
ccode(diff(alphaH, vQt_y))
ccode(diff(alphaH, vQt_z))

disp('d alphaV / dP')
ccode(diff(alphaV, vPt_x))
ccode(diff(alphaV, vPt_y))
ccode(diff(alphaV, vPt_z))
disp('d alphaV / dQ')
ccode(diff(alphaV, vQt_w))
ccode(diff(alphaV, vQt_x))
ccode(diff(alphaV, vQt_y))
ccode(diff(alphaV, vQt_z))

  t0 = -(((vQt_x*vRl_12*2.0+vQt_y*vRl_22*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_12*-4.0+vQt_w*vRl_22*2.0+vQt_x*vRl_32*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_12*2.0+vQt_z*vRl_22*4.0-vQt_y*vRl_32*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_x*vRl_13*2.0+vQt_y*vRl_23*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_13*-4.0+vQt_w*vRl_23*2.0+vQt_x*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_13*2.0+vQt_z*vRl_23*4.0-vQt_y*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0)+(tan(tilt)*((vQt_x*vRl_11*2.0+vQt_y*vRl_21*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_11*-4.0+vQt_w*vRl_21*2.0+vQt_x*vRl_31*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_11*2.0+vQt_z*vRl_21*4.0-vQt_y*vRl_31*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))+curve*((vQt_x*vRl_11*2.0+vQt_y*vRl_21*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_11*-4.0+vQt_w*vRl_21*2.0+vQt_x*vRl_31*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_11*2.0+vQt_z*vRl_21*4.0-vQt_y*vRl_31*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*2.0+(gib_mag*cos(gib_phase+atan((vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))))*(((vQt_x*vRl_12*2.0+vQt_y*vRl_22*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_12*-4.0+vQt_w*vRl_22*2.0+vQt_x*vRl_32*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_12*2.0+vQt_z*vRl_22*4.0-vQt_y*vRl_32*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))/(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))-((vQt_x*vRl_13*2.0+vQt_y*vRl_23*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_13*-4.0+vQt_w*vRl_23*2.0+vQt_x*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_13*2.0+vQt_z*vRl_23*4.0-vQt_y*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))*(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)))/(pow(vPl_x*vRl_12+vPl_y*vRl_22+vPl_z*vRl_32-vPt_x*vRl_12-vPt_y*vRl_22-vPt_z*vRl_32+(vRl_12*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_22*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_32*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_22*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_12*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_32*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_32*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_12*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_22*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)+1.0)-curve*((vQt_x*vRl_13*2.0+vQt_y*vRl_23*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_13*-4.0+vQt_w*vRl_23*2.0+vQt_x*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_13*2.0+vQt_z*vRl_23*4.0-vQt_y*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))*pow(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0)*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),3.0)*2.0-tan(tilt)*((vQt_x*vRl_13*2.0+vQt_y*vRl_23*2.0)*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z)+(vQt_z*vRl_13*-4.0+vQt_w*vRl_23*2.0+vQt_x*vRl_33*2.0)*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)-(vQt_w*vRl_13*2.0+vQt_z*vRl_23*4.0-vQt_y*vRl_33*2.0)*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z))*(vPl_x*vRl_11+vPl_y*vRl_21+vPl_z*vRl_31-vPt_x*vRl_11-vPt_y*vRl_21-vPt_z*vRl_31+(vRl_11*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_21*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_31*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_21*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_11*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_31*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_31*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_11*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_21*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z))*1.0/pow(vPl_x*vRl_13+vPl_y*vRl_23+vPl_z*vRl_33-vPt_x*vRl_13-vPt_y*vRl_23-vPt_z*vRl_33+(vRl_13*((vQt_y*vQt_y)*2.0+(vQt_z*vQt_z)*2.0-1.0)-vRl_23*(vQt_w*vQt_z*2.0+vQt_x*vQt_y*2.0)+vRl_33*(vQt_w*vQt_y*2.0-vQt_x*vQt_z*2.0))*(tPtl_x+tRtl_11*tlPs_x+tRtl_12*tlPs_y+tRtl_13*tlPs_z)+(vRl_23*((vQt_x*vQt_x)*2.0+(vQt_z*vQt_z)*2.0-1.0)+vRl_13*(vQt_w*vQt_z*2.0-vQt_x*vQt_y*2.0)-vRl_33*(vQt_w*vQt_x*2.0+vQt_y*vQt_z*2.0))*(tPtl_y+tRtl_21*tlPs_x+tRtl_22*tlPs_y+tRtl_23*tlPs_z)+(vRl_33*((vQt_x*vQt_x)*2.0+(vQt_y*vQt_y)*2.0-1.0)-vRl_13*(vQt_w*vQt_y*2.0+vQt_x*vQt_z*2.0)+vRl_23*(vQt_w*vQt_x*2.0-vQt_y*vQt_z*2.0))*(tPtl_z+tRtl_31*tlPs_x+tRtl_32*tlPs_y+tRtl_33*tlPs_z),2.0);
