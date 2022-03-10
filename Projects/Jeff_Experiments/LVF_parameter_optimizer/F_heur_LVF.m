function F_est = F_heur_LVF(alphap,dRotNorm,fa,vmax,wandamax,wmax)
%F_HEUR_LVF
%    F_EST = F_HEUR_LVF(ALPHAP,DROTNORM,FA,VMAX,WANDAMAX,WMAX)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    31-Mar-2021 12:42:06

t2 = 1.0./fa;
t3 = 1.0./vmax;
F_est = alphap.*t2.*(wmax.*pi.*2.0+dRotNorm.*t3.*(8.31e+2./1.0e+2)+(fa.*vmax.*2.0)./alphap+alphap.*t2.*t3.*wandamax.*(2.61e+2./2.0e+1));