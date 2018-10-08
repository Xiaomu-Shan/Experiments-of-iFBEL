clear; 
% clc;
tic;

%%
%%%%    机械臂固定参数    %%%%

m1=2.23; m2=5.28; m3=14.79; m4=5.28; m5=2.23; m6=1.3;
l1=0.332; l2=0.302; l3=0.486; l4=0.302; l5=0.332; 
d1=0.189; d2=0.236; d3=0.486; d4=0.236; d5=0.189;
I1=3.3; I2=3.3; I3=3.3; I4=3.3; I5=3.3; 
b=0.145;
grv = 9.8;

%%%%%%%%%%%%%
det11=m1*d1^2+(m2+m3+m4+m5+m6)*l1^2+I1;
det22=m2*d2^2+(m3+m4+m5+m6)*l2^2+I2;
det33=m3*d3^2+I3;
det44=m4*(l4-d4)^2+(m5+m6)*d4^2+I4;
det55=m5*(l5-d5)^2+m6*l5^2+I5;
det66=m6*b^2+I5;
det12=m2*l1*d2+(m3+m4+m5+m6)*l1*l2;
det13=m3*l1*d3;
det14=-m4*l1*(l4-d4)-(m5+m6)*l1*l4;
det15=-m5*l1*(l5-d5)-m6*l1*l5;
det16=-m6*l1*b;
det23=m3*l2*d3;
det24=-m4*l2*(l4-d4)-(m5+m6)*l2*l4;
det25=-m5*l2*(l5-d5)-m6*l2*l5;
det26=-m6*l2*b;
det34=0;det35=0;det36=0;
det45=m5*l4*(l5-d5)+m6*l4*l5;
det46=m6*l4*b;
det56=m6*l5*b;
det21=det12;det31=det13;det41=det14;det51=det15;det61=det16;
det32=det23;det42=det24;det52=det25;det62=det26;
det43=det34;det53=det35;det63=det36;
det54=det45;det64=det46;det65=det56;

h1=(m1*d1+m2*l1+m3*l1+m4*l1+m5*l1+m6*l1)*grv;
h2=(m2*d2+m3*l2+m4*l2+m5*l2+m6*l2)*grv;
h3=m3*d3*grv;
h4=(m4*d4-m4*l4-m5*l4-m6*l4)*grv;
h5=(m5*d5-m5*l5-m6*l5)*grv;
h6=-m6*b*grv;

%%%%    初始角度    %%%%
teda1=0.37;
teda2=0.5;
teda3=0.75;
teda4=-0.15;
teda5=-0.56;
teda6=0.85;

th1=0;
th2=0;
th3=0;
th4=0;
th5=0;
th6=0;

b1=0.2175;
b2=0;
b3=0;
u=0;
v=0;
w=0;

TH1=[];
TH2=[];
TH3=[];
TH4=[];
TH5=[];
TH6=[];

 q_old=[teda1 teda2 teda3 teda4 teda5 teda6]';  %% initial value
% q_old=[0 0 0 0 0 0]';
qd_old=[0 0 0 0 0 0]';
qdd_old=[0 0 0 0 0 0]';
error=[0 0 0 0 0 0]';
ierror=[0 0 0 0 0 0]';
error_old=[0 0 0 0 0 0]';
derror = [0 0 0 0 0 0]';
ss=[0 0 0 0 0 0]';
U=[0 0 0 0 0 0];
Tao = [0 0 0 0 0 0]';

Q=q_old;
qd=qd_old;
qdd=qdd_old;

%%
%%%%%    FCMAC    %%%%
int_d = 6;
out_d = 6;
block_num = 8;
mns = ones(int_d,1)*linspace(-1.4,1.4,block_num);
v_std = 0.01*ones(6,8);
wght = zeros(6,8); 
% wght = rand(6,8);
r_w = zeros(6,8);
u_exp = zeros(6,8);
u_exp_delay = zeros(6,8);
pru = r_w.*u_exp_delay;
Fi = zeros(1,8);
Fi_m = zeros(6,8);
Fi_v = zeros(6,8);
Fi_r = zeros(6,8);
Fi_Mn = zeros(8,48);
Fi_Vs = zeros(8,48);
Fi_Rw = zeros(8,48);
output = zeros(1,6);

%%%%%%%%%%   BELC    %%%%%%%%%%%%%
block_num_BELC = 8;         %%%%%%%%可以改
m_BE = ones(int_d,1)*linspace(-1.4,1.4,block_num_BELC);
v_BE = 0.1*ones(int_d,block_num);
w_BE = zeros(int_d,block_num,out_d);
%  w_BE = rand(int_d,block_num,out_d);
exp_BE = zeros(int_d,block_num);

a_w = zeros(out_d,1);%大脑输出
o_w = zeros(out_d,1);%FCMAC输出

Rd_p = zeros(out_d,1);   %更新
w_BE_d = zeros(int_d,block_num_BELC,out_d);

% % % % % % % % % % % learning rate % % % % % % % % % % % % % % 

alpha = 0.01;
b_gains = 0.1*ones(1,int_d);
c_gains = 0.1*ones(1,out_d);
etaw=1*eye(8);
etam=100*eye(48);
etav=100*eye(48);
% etar=0.001*eye(48);

RR=0.1*eye(6);

%%
% % plot parameter % %
REF1=[ ];
Q1=[ ];
E1=[ ];
REF2=[ ];
Q2=[ ];
E2=[ ];
REF3=[ ];
Q3=[ ];
E3=[ ];
REF4=[ ];
Q4=[ ];
E4=[ ];
REF5=[ ];
Q5=[ ];
E5=[ ];
REF6=[ ];
Q6=[ ];
E6=[ ];

TT=[ ];
SS=[];

U1=[];U2=[];U3=[];U4=[];U5=[];U6=[];
Un1=[];Un2=[];Un3=[];Un4=[];Un5=[];Un6=[];
Urc1=[];Urc2=[];Urc3=[];Urc4=[];Urc5=[];Urc6=[];
Ucmac = [];

%%
RMSE = zeros(out_d,1);
beg = -1;
ts = 0.001;
Allts = 1;
for t=beg:ts:Allts
 %  error_old=error;
    Q=q_old;
    qd=qd_old;
    qdd=qdd_old;
    
    s11 = sin(Q(1)-Q(1)); s12 = sin(Q(1)-Q(2)); s13 = sin(Q(1)-Q(3)); s14 = sin(Q(1)-Q(4)); s15 = sin(Q(1)-Q(5)); s16 = sin(Q(1)-Q(6));
    s21 = sin(Q(2)-Q(1)); s22 = sin(Q(2)-Q(2)); s23 = sin(Q(2)-Q(3)); s24 = sin(Q(2)-Q(4)); s25 = sin(Q(2)-Q(5)); s26 = sin(Q(2)-Q(6));
    s31 = sin(Q(3)-Q(1)); s32 = sin(Q(3)-Q(2)); s33 = sin(Q(3)-Q(3)); s34 = sin(Q(3)-Q(4)); s35 = sin(Q(3)-Q(5)); s36 = sin(Q(3)-Q(6));
    s41 = sin(Q(4)-Q(1)); s42 = sin(Q(4)-Q(2)); s43 = sin(Q(4)-Q(3)); s44 = sin(Q(4)-Q(4)); s45 = sin(Q(4)-Q(5)); s46 = sin(Q(4)-Q(6));
    s51 = sin(Q(5)-Q(1)); s52 = sin(Q(5)-Q(2)); s53 = sin(Q(5)-Q(3)); s54 = sin(Q(5)-Q(4)); s55 = sin(Q(5)-Q(5)); s56 = sin(Q(5)-Q(6));
    s61 = sin(Q(6)-Q(1)); s62 = sin(Q(6)-Q(2)); s63 = sin(Q(6)-Q(3)); s64 = sin(Q(6)-Q(4)); s65 = sin(Q(6)-Q(5)); s66 = sin(Q(6)-Q(6));
    
    c11 = cos(Q(1)-Q(1)); c12 = cos(Q(1)-Q(2)); c13 = cos(Q(1)-Q(3)); c14 = cos(Q(1)-Q(4)); c15 = cos(Q(1)-Q(5)); c16 = cos(Q(1)-Q(6)); 
    c21 = cos(Q(2)-Q(1)); c22 = cos(Q(2)-Q(2)); c23 = cos(Q(2)-Q(3)); c24 = cos(Q(2)-Q(4)); c25 = cos(Q(2)-Q(5)); c26 = cos(Q(2)-Q(6)); 
    c31 = cos(Q(3)-Q(1)); c32 = cos(Q(3)-Q(2)); c33 = cos(Q(3)-Q(3)); c34 = cos(Q(3)-Q(4)); c35 = cos(Q(3)-Q(5)); c36 = cos(Q(3)-Q(6)); 
    c41 = cos(Q(4)-Q(1)); c42 = cos(Q(4)-Q(2)); c43 = cos(Q(4)-Q(3)); c44 = cos(Q(4)-Q(4)); c45 = cos(Q(4)-Q(5)); c46 = cos(Q(4)-Q(6)); 
    c51 = cos(Q(5)-Q(1)); c52 = cos(Q(5)-Q(2)); c53 = cos(Q(5)-Q(3)); c54 = cos(Q(5)-Q(4)); c55 = cos(Q(5)-Q(5)); c56 = cos(Q(5)-Q(6)); 
    c61 = cos(Q(6)-Q(1)); c62 = cos(Q(6)-Q(2)); c63 = cos(Q(6)-Q(3)); c64 = cos(Q(6)-Q(4)); c65 = cos(Q(6)-Q(5)); c66 = cos(Q(6)-Q(6)); 
    
    s1 = sin(Q(1)); s2 = sin(Q(2)); s3 = sin(Q(3)); s4 = sin(Q(4)); s5 = sin(Q(5)); s6 = sin(Q(6)); 
    
   %%%%%%%%%%%%%%  步态分析  %%%%%%%%%%%%%%%%%  
    th6=-0.6*t-0.7;
    th2=acos((t^2+7)/18);
    th1=asin(-3*sin(th2)/sqrt(t^2+25))-atan(0.2*t);
    th3=-th1-th2;
 
    b2=0.0345+0.066*sin(th6)+0.022*cos(th6);
    b3=-0.022*sin(th6);
    u=0.2*b2*t+1.2*b3;
    v=-0.2*b3*t+1.2*b2;
    w=b2^3+b3^2-b1^2+0.01*t^2+0.36;
 
    th5=-th4+acos(w/sqrt(u^2+v^2))+atan(u/v);
    th4=asin(5*(0.1*t-b2*sin(th4+th5)+b3*cos(th4+th5)));
     
 %   ref = [th1 th2 th3 th4 th5 th6]';
    
%     if t <5*pi
%         ref = [4*sin(2*t)-0.7 4*sin(2*t)-0.7 4*sin(2*t)-0.7]';
%     else
         ref = [th1 th2 th3 -2*t-0.5 cos(t)+2*sin(0.5*t) -0.6*t-0.7]';
%     end
%     if t >5*pi
%         ref = [sin(2*t) sin(2*t) sin(2*t)]';
%     else
%         ref = [-sin(2*t) -2-2*sin(2*t) -2*sin(2*t)]';
%     end
%     if t >18
%         ref = [sin(2*t) sin(2*t) sin(2*t)]';/
%     else
%         ref = [sin(2*t) 0.5+0.2*(sin(t)+sin(2*t)) 0.13-0.1*(sin(t)+sin(2*t))]';
%     end
%     ref = [sin(2*t) sin(2*t) sin(2*t)]';
%     if t >20
%         ref = [sin(2*t)+cos(t+1) 0.3*sin(2*t) 0.2*cos(2*t) sin(2*t) 0.8*sin(2*t)*cos(t+1) 0.2*(cos(2*t)-sin(t))]';
%     else
%         ref = [sin(t+2.5)+0.7*cos(2*t+1.5) 0.2*(sin(t)+sin(2*t)) 0.13-0.1*(sin(t)+sin(2*t)) 0.5*(sin(t)+sin(2*t)) sin(t)+cos(2*t) cos(t)]';
%     end
%    if t >20
%        ref = 1*[sin(2*t) 0.3*sin(t) cos(2*t) cos(t+1) 0.3*sin(t) cos(t+1)]';
%    else
%      %  ref=[1 0 -2 3 0 -1]';
%        ref = 1*[cos(t) cos(t) cos(t) cos(t) cos(t) cos(t)]';
%    end
%     if t >15
%         ref = [0.5*sin(2*t)+cos(t+1) 0.8*sin(2*t)*cos(t+1) 0.2*(cos(2*t)-sin(t))]';
%     else
%         ref = [0.5*(sin(t)+sin(2*t)) sin(t)+cos(2*t) 0.13-0.1*(sin(t)+sin(2*t))]';
%     end
%     if t >15
%         ref = 0.5*[0.5*(sin(2*t)+cos(t+1)) sin(2*t)*cos(t+1) (cos(2*t)-sin(t))]';
%     else
%         ref = 0.5*[0.5*(sin(t+2.5)+0.7*cos(2*t+1.5)) (sin(t)+sin(2*t)) 0.13-(sin(t)+sin(2*t))]';
%     end
%     ref = [0.5*(sin(t)+sin(2*t)) sin(t)+cos(2*t) 0.13-0.1*(sin(t)+sin(2*t))]';
%     xd_dd = -21.13*xd_d-111.63*xd+111.63*Tao;    
%     xd_d = xd_d + xd_dd*ts;
%     xd = xd + xd_d*ts;
%     ref = xd;
    
    error = ref-Q;
    RMSE = RMSE + error.^2;    %均方误差
    
    derror = (error-error_old)/ts;
    ierror = ierror + error;
    %%%%    SMC    %%%%
%     for si = 1:int_d
%         if error(si) > 2
%             error(si) = 2;
%         elseif error(si) < -2;
%            error(si) = -2;
%         else
%           error(si) = error(si);
%         end          
%     end
    
    ss = 2000*error + 20*derror;
%    ss = 100*error + 10*derror + 5*ierror;
    % % % % % % % % % % % % % % controller % % % % % % % % % % % % % % % % % % % % % 
    %%%%     BELC     %%%%
    
    for i = 1:int_d       
        for j = 1:block_num_BELC           
            exp_BE(i,j) = exp(-0.5*(ss(i)-m_BE(i,j)).^2/(v_BE(i,j).^2));  
        end
    end   
    
    for o_d = 1:out_d
        a_w(o_d) = sum(diag(exp_BE*w_BE(:,:,o_d)'));  
    end 
    
    %%%%    FRCMAC    %%%%
    
    u_exp_delay = u_exp;
    for i = 1:6    %%%%    input
        for j = 1:8           
            u_exp(i,j) = exp(-((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))^2/(v_std(i,j)^2));
        end
    end
    Fi = prod(u_exp);
    output = Fi*wght';
    o_w = output';
    
    uBELFCMAC = a_w - o_w ;
    
  % % compensator
     uRC=(((eye(6)+exp_BE*exp_BE')*RR^2+1*eye(6))/(2*RR^2))*ss;    %%eye(6)返回6*6的单位矩阵
  %  uRC=((eye(6)*RR^2+1*eye(6))/(2*RR^2))*ss;
  
    %%%%    输出力矩
    Tao = uRC + uBELFCMAC;      %  RC+FBELCMAC控制    
%   Tao = uRC - o_w;            %  RC+CMAC控制        
%   Tao = uBELFCMAC;            %  BEL+CMAC            
%   Tao = uRC;                  %  RC控制             
%   Tao = a_w;                  %  BELC控制            
%   Tao = -o_w;                 %  CMAC控制   
   Tao = 5000*error  + 200*derror  ;   %  PD控制              

   
%     for si = 1:int_d
%         if Tao(si) > 10
%             Tao(si) = 10;
%         elseif Tao(si) < -10;
%             Tao(si) = -10;
%         else
%             Tao(si) = Tao(si);
%         end          
%     end

    %%
    % % % % % % % % % % % % % % dynamic fcn 动力学方程 % % % % % % % % % % % % % % % % % % % % % 
    %%%%    惯性矩阵    %%%%
    
    bq11=det11*c11; bq12=det12*c12; bq13=det13*c13; bq14=det14*c14; bq15=det15*c15; bq16=det16*c16;
    bq21=det21*c21; bq22=det22*c22; bq23=det23*c23; bq24=det24*c24; bq25=det25*c25; bq26=det26*c26;
    bq31=det31*c31; bq32=det32*c32; bq33=det33*c33; bq34=det34*c34; bq35=det35*c35; bq36=det36*c36;
    bq41=det41*c41; bq42=det42*c42; bq43=det43*c43; bq44=det44*c44; bq45=det45*c45; bq46=det46*c46;
    bq51=det51*c51; bq52=det52*c52; bq53=det53*c53; bq54=det54*c54; bq55=det55*c55; bq56=det56*c56;
    bq61=det61*c61; bq62=det62*c62; bq63=det63*c63; bq64=det64*c64; bq65=det65*c65; bq66=det66*c66;

    Mq = [bq11 bq12 bq13 bq14 bq15 bq16; bq21 bq22 bq23 bq24 bq25 bq26; bq31 bq32 bq33 bq34 bq35 bq36;
          bq41 bq42 bq43 bq44 bq45 bq46; bq51 bq52 bq53 bq54 bq55 bq56; bq61 bq62 bq63 bq64 bq65 bq66]; 
    %%%%    向心矩阵    %%%%
 
    cq11=det11*s11; cq12=det12*s12; cq13=det13*s13; cq14=det14*s14; cq15=det15*s15; cq16=det16*s16; 
    cq21=det21*s21; cq22=det22*s22; cq23=det23*s23; cq24=det24*s24; cq25=det25*s25; cq26=det26*s26; 
    cq31=det31*s31; cq32=det32*s32; cq33=det33*s33; cq34=det34*s34; cq35=det35*s35; cq36=det36*s36; 
    cq41=det41*s41; cq42=det42*s42; cq43=det43*s43; cq44=det44*s44; cq45=det45*s45; cq46=det46*s46; 
    cq51=det51*s51; cq52=det52*s52; cq53=det53*s53; cq54=det54*s54; cq55=det55*s55; cq56=det56*s56; 
    cq61=det61*s61; cq62=det62*s62; cq63=det63*s63; cq64=det64*s64; cq65=det65*s65; cq66=det66*s66; 
    
    Cq_qd = [cq11 cq12 cq13 cq14 cq15 cq16; cq21 cq22 cq23 cq24 cq25 cq26; cq31 cq32 cq33 cq34 cq35 cq36;
             cq41 cq42 cq43 cq44 cq45 cq46; cq51 cq52 cq53 cq54 cq55 cq56; cq61 cq62 cq63 cq64 cq65 cq66];

    %%%%    重力矩阵    %%%%
    Gq = -1*[h1*s1; h2*s2; h3*s3; h4*s4; h5*s5; h6*s6] ;
          
    %%
    Tao_d = 1500*[exp(-0.1*t);exp(-0.1*t);exp(-0.1*t);exp(-0.1*t);exp(-0.1*t);exp(-0.1*t)];
%   Tao_d = [0 0 0 0 0 0]';
%   Tao_d = 0.3*rand(6,1);
    QDD = inv(Mq)*(Tao-Tao_d-Cq_qd*qd-Gq);      %%  状态方程
    
    qdd_old=QDD;
    qd_old=qd_old+qdd_old*ts;
    q_old=q_old+qd_old*ts;
    
    %%
    % % % % % % % % % % % % % % % learning online  % % % % % % % % % % % % % % % % % % % % % % 
    %%%%     FCMAC      %%%%
    %%%%    权值W更新    %%%%
    dww = -etaw*Fi'*ss';
    
    %%%%    CC  高斯函数均值u更新    %%%%
    for i = 1:6
        for j = 1:8
%             Fi_m(i,j) = 2*Fi(j)*((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))/(v_std(i,j)^2);
            Fi_Mn(j,i+(j-1)*6) = 2*Fi(j)*u_exp(i,j)*((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))/(v_std(i,j)^2);
        end
    end

    m_d = -etam*Fi_Mn'*wght'*ss;

    %%%%    VV  高斯函数标准差cita更新    %%%%
    for i = 1:6
        for j = 1:8
            Fi_Vs(j,i+(j-1)*6) = 2*Fi(j)*u_exp(i,j)*((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))^2/(v_std(i,j)^3);
        end
    end
    v_d = -etav*Fi_Vs'*wght'*ss;
    
    %%%%    R  r_wght更新    %%%%
%    for i = 1:6
%        for j = 1:8
%            Fi_Rw(j,i+(j-1)*6) = -2*Fi(j)*u_exp(i,j)*u_exp_delay(i,j)*((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))/(v_std(i,j)^2);
%        end
%    end
%    r_d = -etar*Fi_Rw'*wght'*ss;
    
    %%%%    
    wght = wght + dww';
    m_d1 = reshape(m_d,6,8);
    v_d1 = reshape(v_d,6,8);
%   r_d1 = reshape(r_d,6,8);
    mns = mns + m_d1;
    v_std = v_std + v_d1;
%   r_w = r_w + r_d1*ts;           %加入反馈结构

    %%%%    BELC更新    %%%%    
    for i = 1:out_d      
        Rd_p(i) = b_gains(i)*ss(i) + c_gains(i)*Tao(i);            
    end  
   
    for i = 1:int_d   
        for j = 1:block_num_BELC
            for o_d = 1:out_d               
                w_BE_d(i,j,o_d) = alpha*(exp_BE(i,j)*(max(0,Rd_p(o_d)-a_w(o_d))));  
            end
        end
    end
    
    w_BE = w_BE + w_BE_d ;    
    
    %%
    % % % % % % % % % % % % % % % plot % % % % % % % % % % % % % % % % % % % % % %    
    Ucmac = [Ucmac uBELFCMAC];
    
    REF1=[REF1 ref(1)];
    Q1=[Q1 Q(1)];
    E1=[E1 error(1)];
    U1=[U1 Tao(1)];

    REF2=[REF2 ref(2)];
    Q2=[Q2 Q(2)];
    E2=[E2 error(2)];
    U2=[U2 Tao(2)];

    REF3=[REF3 ref(3)];
    Q3=[Q3 Q(3)];
    E3=[E3 error(3)];
    U3=[U3 Tao(3)];
    
    REF4=[REF4 ref(4)];
    Q4=[Q4 Q(4)];
    E4=[E4 error(4)];
    U4=[U4 Tao(4)];
    
    REF5=[REF5 ref(5)];
    Q5=[Q5 Q(5)];
    E5=[E5 error(5)];
    U5=[U5 Tao(5)];
    
    REF6=[REF6 ref(6)];
    Q6=[Q6 Q(6)];
    E6=[E6 error(6)];
    U6=[U6 Tao(6)];
    
    Un1 = [Un1 uBELFCMAC(1)];
    Urc1 = [Urc1 uRC(1)];
    Un2 = [Un2 uBELFCMAC(2)];
    Urc2 = [Urc2 uRC(2)];
    Un3 = [Un3 uBELFCMAC(3)];
    Urc3 = [Urc3 uRC(3)];
    Un4 = [Un4 uBELFCMAC(4)];
    Urc4 = [Urc4 uRC(4)];
    Un5 = [Un5 uBELFCMAC(5)];
    Urc5 = [Urc5 uRC(5)];
    Un6 = [Un6 uBELFCMAC(6)];
    Urc6 = [Urc6 uRC(6)];
    
    TT=[TT t];
    
    SS=[SS ss];
    
    error_old = error;

end

 fprintf('The RMSE1 is %d\n',sqrt(sum(E1.^2)/(t/ts)));
 fprintf('The RMSE2 is %d\n',sqrt(sum(E2.^2)/(t/ts)));
 fprintf('The RMSE3 is %d\n',sqrt(sum(E3.^2)/(t/ts)));
 fprintf('The RMSE4 is %d\n',sqrt(sum(E4.^2)/(t/ts)));
 fprintf('The RMSE5 is %d\n',sqrt(sum(E5.^2)/(t/ts)));
 fprintf('The RMSE6 is %d\n',sqrt(sum(E6.^2)/(t/ts)));

figure(1)
    subplot(1,2,1);
    hold on;
    plot(TT,Q1,':k')
       xlabel('time (s)'); ylabel('angle 1 (rad)');axis([beg,Allts,-1,0]);
    subplot(1,2,2);
    hold on;
    plot(TT,E1,':k')
        xlabel('time (s)'); ylabel('error (rad)');axis([beg,Allts,-1,1]);
%     subplot(3,1,3);
%     hold on;
%     plot(TT,U1,'--k')
%         xlabel('time (s)'); ylabel('control effort (rad)');

    
 figure(2)
    subplot(1,2,1);
    hold on;
    plot(TT,Q2,':k')
            xlabel('time (s)'); ylabel('angle 2 (rad)'); axis([beg,Allts,1,2]);
    subplot(1,2,2);
    hold on;
    plot(TT,E2,':k')
            xlabel('time (s)'); ylabel('error (rad)'); axis([beg,Allts,-1,1]);
%     subplot(3,1,3);
%     hold on;
%     plot(TT,U2,'--k')
%             xlabel('time (s)'); ylabel('control effort (rad)'); 

    
 figure(3)
    subplot(1,2,1);
    hold on;
    plot(TT,Q3,':k')
            xlabel('time (s)'); ylabel('angle 3 (rad)'); axis([beg,Allts,-1.5,0]);
    subplot(1,2,2);
    hold on;
    plot(TT,E3,':k')
            xlabel('time (s)'); ylabel('error (rad)'); axis([beg,Allts,-1,1]);
 %    subplot(3,1,3);
 %    hold on;
 %    plot(TT,U3,'--k')
 %            xlabel('time (s)'); ylabel('control effort (rad)');

    
  figure(4)
    subplot(1,2,1);
    hold on;
    plot(TT,Q4,':k')
            xlabel('time (s)'); ylabel('angle 4 (rad)'); axis([beg,Allts,-3,3]);
    subplot(1,2,2);
    hold on;
    plot(TT,E4,':k')
            xlabel('time (s)'); ylabel('error (rad)');axis([beg,Allts,-0.5,0.5]);
%     subplot(3,1,3);
%     hold on;
%     plot(TT,U4,'--k')
%             xlabel('time (s)'); ylabel('control effort (rad)');
             
             
 
 figure(5)
    subplot(1,2,1);
    hold on;
    plot(TT,Q5,':k')
            xlabel('time (s)'); ylabel('angle 5 (rad)'); axis([beg,Allts,-1,2]);
    subplot(1,2,2);
    hold on;
    plot(TT,E5,':k')
            xlabel('time (s)'); ylabel('error (rad)');axis([beg,Allts,-0.3,0.3]);
%     subplot(3,1,3);
%     hold on;
%     plot(TT,U5,'--k')
%             xlabel('time (s)'); ylabel('control effort (rad)');
             
             
 figure(6)
    subplot(1,2,1);
    hold on;
    plot(TT,Q6,':k')
            xlabel('time (s)'); ylabel('angle 6 (rad)'); axis([beg,Allts,-1.5,0]);
    subplot(1,2,2);
    hold on;
    plot(TT,E6,':k')
            xlabel('time (s)'); ylabel('error (rad)');axis([beg,Allts,-0.5,0.5]);
%     subplot(3,1,3);
%     hold on;
%     plot(TT,U6,'--k')
%             xlabel('time (s)'); ylabel('control effort (rad)');
             
             
% figure(7)
%    subplot(2,1,1);
%    hold on;
%    plot(TT,Un1,'r')
%        xlabel('time (s)'); ylabel('Unet (N-m)');
%    subplot(2,1,2)
%    hold on;
%    plot(TT,Urc1,'r')
%        xlabel('time (s)'); ylabel('uRC (N-m)');

    
%    figure(8)
%    subplot(2,1,1);
%    hold on;
%    plot(TT,Un2,'r')
%        xlabel('time (s)'); ylabel('Unet (N-m)');
%    subplot(2,1,2)
%    hold on;
%    plot(TT,Urc2,'r')
%        xlabel('time (s)'); ylabel('uRC (N-m)');
 
    
 %   figure(9)
 %   subplot(2,1,1);
 %   hold on;
 %   plot(TT,Un3,'r')
 %       xlabel('time (s)'); ylabel('Unet (N-m)');
 %   subplot(2,1,2)
 %   hold on;
 %   plot(TT,Urc3,'r')
 %       xlabel('time (s)'); ylabel('uRC (N-m)');
        
 %   figure(10)
 %   subplot(2,1,1);
 %   hold on;
 %   plot(TT,Un4,'r')
 %       xlabel('time (s)'); ylabel('Unet (N-m)');
 %   subplot(2,1,2)
 %   hold on;
 %   plot(TT,Urc4,'r')
 %       xlabel('time (s)'); ylabel('uRC (N-m)');
        
        
 %    figure(11)
 %   subplot(2,1,1);
 %  hold on;
 %   plot(TT,Un5,'r')
 %       xlabel('time (s)'); ylabel('Unet (N-m)');
 %   subplot(2,1,2)
 %   hold on;
 %   plot(TT,Urc5,'r')
 %       xlabel('time (s)'); ylabel('uRC (N-m)');
        
        
 %   figure(12)
 %   subplot(2,1,1);
 %   hold on;
 %   plot(TT,Un6,'r')
 %       xlabel('time (s)'); ylabel('Unet (N-m)');
 %   subplot(2,1,2)
 %   hold on;
 %   plot(TT,Urc6,'r')
 %       xlabel('time (s)'); ylabel('uRC (N-m)');
     
    RMSE = RMSE./(1+Allts/ts);
toc;