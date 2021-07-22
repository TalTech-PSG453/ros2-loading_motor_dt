function [ids, iqs] =  parkclarke(ch_Current_L1__brown__interval_1, ch_Current_L2__black__interval_1, ch_Current_L3__grey__interval_1)

%i=1;
%name = sprintf('%d.mat',i)
%load(name);
a1=(-0.5 + (j*sqrt(3)/2));
a2=(-0.5 - (j*sqrt(3)/2));
ia=ch_Current_L1__brown__interval_1;
Ia=1.414*rms(ia);
iap=(Ia*cos(0))+ j*(Ia*sin(0));
ib=ch_Current_L2__black__interval_1;
Ib=1.414*rms(ib);
ibp=Ib*cos(2*pi/3)+ j*Ib*sin(2*pi/3);
ic=ch_Current_L3__grey__interval_1;
Ic=1.414*rms(ic);
icp=Ic*cos(4*pi/3)+ j*Ic*sin(4*pi/3);


ids=(2*ia/3)-(ib/3)-(ic/3);
iqs=(ib/sqrt(3))-(ic/sqrt(3));
is=sqrt((ids).^2+(iqs).^2);
is1=is-mean(is); %taken across zero line


I0 =abs((iap+ibp+icp)/3);         % Zero sequence currents
In =abs((iap+(a1*ibp)+a2*icp)/3);   % Negative sequence currents
Ip =abs((iap+(a2*ibp)+a1*icp)/3);   % Positive sequence currents

end
