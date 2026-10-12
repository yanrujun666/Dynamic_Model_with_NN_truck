function [Fx0,Fy0,Fx,Fy,Mz] = tire(Fz,camber,lamda,alpha,f)
if Fz>0
%Fz 轮胎垂向载荷 N
%camber 外倾角 rad
%lamda 纵向滑移率
%alpha 车辆侧偏角 rad
alpha=tan(alpha);
camber=sin(camber);
Fz0=4100; %名义垂向载荷N
dfz=(Fz-Fz0)/Fz0;
%% Pure_Fx
%轮胎参数——纵向力
pcx1=1.62;
pdx1=1.035; pdx2=-0.0487; pdx3=-2.27;
pex1=0.5; pex2=-0.122;pex3=-0.063; pex4=0;
pkx1=19.4; pkx2=-0.13; pkx3=0.171;
pvx1=0; pvx2=0;
phx1=-0.0005; phx2=0.0000842;
%pure_Fx 魔术公式
Cx=pcx1;
Shx=phx1+phx2*dfz;
lamda_x=lamda+Shx;
Dx=(pdx1+pdx2*dfz)*(1-pdx3*camber*camber)*Fz;
Ex=(pex1+pex2*dfz+pex3*dfz*dfz)*(1-pex4*sign(lamda_x));
Kx=Fz*(pkx1+pkx2*dfz)*exp(pkx3*dfz);
Bx=Kx/(Cx*Dx);
Svx=Fz*(pvx1+pvx2*dfz);
Fx0=Dx*sin(Cx*atan(Bx*lamda_x-Ex*(Bx*lamda_x-atan(Bx*lamda_x))))+Svx;
%% 联合工况纵向力Fx
%轮胎参数——纵向力
rbx1=9.0;rbx2=-8.75;
rcx1=1.125;
rex1=0.078;rex2=-0.16;
rhx1=-0.03;
%纵向力加权函数
Bxa=rbx1*cos(atan(rbx2*lamda));
Cxa=rcx1;
Exa=rex1+rex2*dfz;
Shxa=rhx1;
alpha_s=alpha+Shxa;
Gxa=cos(Cxa*atan(Bxa*alpha_s-Exa*(Bxa*alpha_s-atan(Bxa*alpha_s))))...
/cos(Cxa*atan(Bxa*Shxa-Exa*(Bxa*Shxa-atan(Bxa*Shxa))));
Fx=Fx0*Gxa;
%% Pure_Fy
%轮胎参数——侧向力
pcy1=1.29;
pdy1=-0.9; pdy2=0.18; pdy3=-4.5;
pey1=-1.07; pey2=0.68;pey3=-0.63; pey4=-12.35;
pky1=-12.95; pky2=1.72; pky3=0.22;
pvy1=0.045; pvy2=-0.03;pvy3=-0.174;pvy4=-0.45;
phy1=0.0035; phy2=-0.003;phy3=0.045;
%pure_Fy 魔术公式
Cy=pcy1;
Shy=phy1+phy2*dfz+phy3*camber;
alpha_y=alpha+Shy;
uy=(pdy1+pdy2*dfz)*(1-pdy3*camber*camber);
Dy=uy*Fz;
Ey=(pey1+pey2*dfz)*(1-(pey3+pey4*camber)*sign(alpha_y));
Ky=pky1*Fz0*sin(2*atan(Fz/(pky2*Fz0)))*(1-pky3*abs(camber));
By=Ky/(Cy*Dy);
Svy=Fz*(pvy1+pvy2*dfz+(pvy3+pvy4*dfz)*camber);
Fy0=Dy*sin(Cy*atan(By*alpha_y-Ey*(By*alpha_y-atan(By*alpha_y))))+Svy;
%% 联合工况侧向力Fy
%轮胎参数——侧向力
rby1=6.38;rby2=7.95;rby3=-0.06;
rcy1=1.10;
rey1=0.23;rey2=0.41;
rhy1=0.0007;rhy2=0.024;
rvy1=0;rvy2=0;rvy3=0;rvy4=10;rvy5=1.95;rvy6=-50;
%侧向力加权函数
Bya=rby1*cos(atan(rby2*(alpha-rby3)));
Cya=rcy1;
Eya=rey1+rey2*dfz;
Shya=rhy1+rhy2*dfz;
lamda_s=lamda+Shya;
Gya=cos(Cya*atan(Bya*lamda_s-Eya*(Bya*lamda_s-atan(Bya*lamda_s))))/cos(Cya*atan(Bya*Shya-Eya*(Bya*Shya-atan(Bya*Shya))));
%轮胎参数——侧向力
Svyr=uy*Fz*(rvy1+rvy2*dfz+rvy3*camber)*cos(atan(rvy4*alpha))*sin(rvy5*atan(rvy6*lamda));
Fy=Fy0*Gya+Svyr;

%% Pure_Mz
%轮胎参数——回正力矩
R0=0.287; %轮胎空载半径/m
qbz1=8.37;qbz2=-2.92;qbz3=0.48;qbz4=-0.45;qbz5=-0.44;
qcz1=1.19;
qdz1=0.11;qdz2=-0.003;qdz3=-0.56;qdz4=8.40;
qez1=-2.9;qez2=-0.55;qez3=0;qez4=-0.13;qez5=-3.68;
%pure_Mz0 魔术公式
Ct=qcz1;
alpha_t=alpha;
Bt=(qbz1+qbz2*dfz+qbz3*dfz^2)*(1+qbz4*camber+qbz5*abs(camber));
Dt=Fz*(qdz1+qdz2*dfz)*(1+qdz3*camber+qdz4*camber^2)*R0/Fz0;
Et=(qez1+qez2*dfz+qez3*dfz^2)*(1+(qez4+qez5*camber)*((2/pi)*atan(Bt*Ct*alpha_t)));
t=Dt*cos(Ct*atan(Bt*alpha_t-Et*(Bt*alpha_t-atan(Bt*alpha_t))))*cos(alpha); %轮胎拖距
%轮胎参数——回正力矩
% qbz9=60;qbz10=1.177895045;
% qdz6=0.000688243;qdz7=0.002750679;qdz8=-0.303576739;qdz9=0.007781541;
%pure_Mz0 魔术公式
% Br=(qbz9+qbz10*By*Cy);
% Cr=1;
% Dr=Fz*(qdz6+qdz7*dfz+(qdz8+qdz9*dfz)*camber)*R0;
% alpha_r=alpha;
%Mzr=Dr*cos(Cr*atan(Br*alpha_r))*cos(alpha); %残余力矩
Mz=-t*Fy0;
else
    Fx0=0;
    Fy0=0;
    Fx=0;
    Fy=0;
    Mz=0;
end












