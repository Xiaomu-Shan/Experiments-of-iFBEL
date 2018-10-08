clear; 
% clc;
tic;

%%
%%%%    机械臂固定参数    %%%%
% inrt = [43.33e-3, 25.08e-3, 32.67e-3];
% mass = [1.2, 1.5, 3.0];
% % mass = [3.0, 2.0, 3.0];
% a_lth = [0.5, 0.4, 0.3];
% 
% d1 = 0.5*((0.25*mass(1)+mass(2)+mass(3))*a_lth(1)^2+inrt(1));
% d2 = 0.5*((0.25*mass(2)+mass(3))*a_lth(2)^2+inrt(2));
% d3 = 0.5*((0.25*mass(3))*a_lth(3)^2+inrt(3));
% d4 = (0.5*mass(2)+mass(3))*a_lth(1)*a_lth(2);
% d5 = 0.5*mass(3)*a_lth(1)*a_lth(3);
% d6 = 0.5*mass(3)*a_lth(2)*a_lth(3);
a1 = 0.6; a2 = 0.5; a3 = 0.4;
l1 = 1/2*a1; l2 = 1/2*a2; l3 = 1/2*a3;
ml1 = 3; ml2 = 1.8; ml3 = 1.5;
mm1 = 0.6; mm2 = 1; mm3 = 0.6;
Il1 = 50.45e-3; Il2 = 32.68e-3; Il3 = 30.47e-3;
Im1 = 12.0e-3; Im2 = 12.0e-3; Im3 = 12.0e-3; 
kr1 = 1; kr2 = 1; kr3 = 1;
grv = 9.8;

%%%%    初始角度    %%%%
teda1=-0.3;
teda2=0.1;
teda3=-0.4;

q_old=[teda1 teda2 teda3]';  %% initial value
qd_old=[0 0 0]';
qdd_old=[0 0 0]';
xd = [0 0 0]';
xd_d = [0 0 0]';
xd_dd = [0 0 0]';

error=[0 0 0]';
error_old=[0 0 0]';
derror = [0 0 0]';
ss=[0 0 0]';
U=[0 0 0];
Tao = [0 0 0]';

Q=q_old;
qd=qd_old;
qdd=qdd_old;

%%
%%%%%    RCMAC    %%%%
int_d = 3;
out_d = 3;
block_num = 8;
mns = ones(int_d,1)*linspace(-2.1,2.1,block_num);
v_std = 1.2*ones(3,8);
wght = zeros(3,8); %%%%    3 output
% wght = rand(3,8);
r_w = zeros(3,8);
u_exp = zeros(3,8);
u_exp_delay = zeros(3,8);
pru = r_w.*u_exp_delay;
Fi = zeros(1,8);
Fi_m = zeros(3,8);
Fi_v = zeros(3,8);
Fi_r = zeros(3,8);
Fi_Mn = zeros(8,24);
Fi_Vs = zeros(8,24);
Fi_Rw = zeros(8,24);
output = zeros(1,3);

% % % % % % % % % % % learning rate % % % % % % % % % % % % % % 

etaw=20*eye(8);%80
etam=0.001*eye(24);
etav=0.001*eye(24);
etar=0.001*eye(24);

RR=0.075*eye(3);%%25

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
TT=[ ];

Q4=[ ];
Q5=[ ];
Q6=[ ];

E4=[ ];
E5=[ ];
E6=[ ];

U1=[];U2=[];U3=[];SS=[];
Un1=[];Un2=[];Un3=[];
Urc1=[];Urc2=[];Urc3=[];
Ucmac = [];
% q=[teda1 teda2 teda3 teda4 teda5 teda6]';

FKK=[];

%%
RMSE = zeros(out_d,1);
Allts = 30;
ts = 0.001;
for t=0:ts:Allts
    
%     error_old=error;
    Q=q_old;
    qd=qd_old;
    qdd=qdd_old;
    
    s1 = sin(Q(1)); s2 = sin(Q(2)); s3 = sin(Q(3));
    c1 = cos(Q(1)); c2 = cos(Q(2)); c3 = cos(Q(3));
    s23 = sin(Q(2) + Q(3)); c23 = cos(Q(2) + Q(3));
%     if t <5*pi
%         ref = [4*sin(2*t)-0.7 4*sin(2*t)-0.7 4*sin(2*t)-0.7]';
%     else
%         ref = [4*sin(2*t)-0.7 -2*sin(2*t)+0.7 -2*sin(2*t)+0.7]';
%     end
%     if t >5*pi
%         ref = [sin(2*t) sin(2*t) sin(2*t)]';
%     else
%         ref = [-sin(2*t) -2-2*sin(2*t) -2*sin(2*t)]';
%     end
%     if t >18
%         ref = [sin(2*t) sin(2*t) sin(2*t)]';
%     else
%         ref = [sin(2*t) 0.5+0.2*(sin(t)+sin(2*t)) 0.13-0.1*(sin(t)+sin(2*t))]';
%     end
%     ref = [sin(2*t) sin(2*t) sin(2*t)]';
%     if t >18
%         ref = [sin(2*t)+cos(t+1) 0.3*sin(2*t)*cos(t+1) 0.2*(cos(2*t)-sin(t))]';
%     else
%         ref = [sin(t+2.5)+0.7*cos(2*t+1.5) 0.5+0.2*(sin(t)+sin(2*t)) 0.13-0.1*(sin(t)+sin(2*t))]';
%     end
    if t >15
        ref = 0.5*[sin(2*t)+cos(t+1) 0.3*sin(2*t)*cos(t+1) 0.2*(cos(2*t)-sin(t))]';
    else
        ref = 0.5*[sin(t+2.5)+0.7*cos(2*t+1.5) 0+0.2*(sin(t)+sin(2*t)) 0.13-0.1*(sin(t)+sin(2*t))]';
    end
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
    RMSE = RMSE + error.^2;
    if t ~= 0
        derror = (error-error_old)/ts;
    end
    %%%%    SMC    %%%%
    ss = 0.55*derror+10*error;
%     ss = ss.*[0 0 1]';
    
    % % % % % % % % % % % % % % controller % % % % % % % % % % % % % % % % % % % % % 
    %%%%    CMAC    %%%%
    u_exp_delay = u_exp;
    for i = 1:3    %%%%    input
        for j = 1:8           
            u_exp(i,j) = exp(-((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))^2/(v_std(i,j)^2));
        end
    end
    Fi = prod(u_exp);
    output = Fi*wght';
    uCMAC = output';
    
    % % compensator
    uRC=((RR^2+eye(3))/(2*RR^2))*ss;    %%eye(6)返回6*6的单位矩阵
    Tao = uRC + uCMAC;    %%%%    输出力矩
  %  Tao = uRC;
%     for si = 1:int_d
%         if Tao(si) > 1000
%             Tao(si) = 1000
%         elseif Tao(si) < -1000;
%             Tao(si) = -1000;
%         else
%             Tao(si) = Tao(si);
%         end          
%     end
%     Tao = uRC;
    if t == 0.1
        pppp=1;
    end 
    %%
    % % % % % % % % % % % % % % dynamic fcn 动力学方程 % % % % % % % % % % % % % % % % % % % % % 
    %%%%    惯性矩阵    %%%%
    bq11 = Il1 + Im2 + Il2 + Il3 + Im3 + Im1*kr1^2 + ml3*(a2*c2 + l3*c23)^2 + a2^2*c2^2*mm3 + c2^2*l2^2*ml2;
    bq12 = 0;
    bq13 = 0;
    bq21 = 0;
    bq22 = ml3*(a2^2+l3^2+2*a2*l3*c3) + a2^2*mm3 + l2^2*ml2 + Il2 + Il3 + Im3 + Im2*kr2^2;
    bq23 = Il3 + Im3*kr3 + a2*l3*ml3*c3 + l3^2*ml3;
    bq31 = 0;
    bq32 = Im3*kr3 + Il3 + a2*l3*ml3*c3 + l3^2*ml3;
    bq33 = l3^2*ml3 + Il3 + Im3*kr3^2;
    Mq = [bq11 bq12 bq13; bq21 bq22 bq23; bq31 bq32 bq33]; %% Bq
    %%%%    向心矩阵    %%%%
    c111 = 0;
    c112 = 1/2*(2*ml3*(a2*c2 + l3*c23)*(-a2*s2-l3*s23)-2*(a2^2*mm3+l2^2*ml2)*s2);
    c113 = 1/2*(2*ml3*(a2*c2 + l3*c23)*l3*(-s23));
    c121 = c112;
    c122 = 0;
    c123 = 0;
    c131 = c113;
    c132 = c123;
    c133 = 0;
    c211 = -c112;
    c212 = -c122;
    c213 = -c123;
    c221 = -c122;
    c222 = 0;
    c223 = 1/2*ml3*(-2*a2*l3*s3);
    c231 = c213;
    c232 = c223;
    c233 = -a2*l3*ml3*s3;
    c311 = -c113;
    c312 = c213;
    c313 = 0;
    c321 = c312;
    c322 =  -c223;
    c323 = 0;
    c331 = 0;
    c332 = 0;
    c333 = 0;

    cq11 = c111*qd(1) + c112*qd(2) + c113*qd(3);
    cq12 = c121*qd(1) + c122*qd(2) + c123*qd(3);
    cq13 = c131*qd(1) + c132*qd(2) + c133*qd(3);
    cq21 = c211*qd(1) + c212*qd(2) + c213*qd(3);
    cq22 = c221*qd(1) + c222*qd(2) + c223*qd(3);
    cq23 = c231*qd(1) + c232*qd(2) + c233*qd(3);
    cq31 = c311*qd(1) + c312*qd(2) + c313*qd(3);
    cq32 = c321*qd(1) + c322*qd(2) + c323*qd(3);
    cq33 = c331*qd(1) + c332*qd(2) + c333*qd(3);
    
    Cq_qd = [cq11 cq12 cq13; cq21 cq22 cq23; cq31 cq32 cq33];

    %%%%    重力矩阵    %%%%
    Gq = [0;
          grv*ml3*(a2*c2 + c23*l3) + c2*grv*l2*ml2 + a2*c2*grv*mm3;
          c23*grv*l3*ml3];
    %%
%     Tao_d = 1*[0.2*sin(6*t);0.1*cos(6*t);0.1*sin(6*t)];
    Tao_d = 20*[0.2*sin(2*t);0.1*cos(2*t);0.1*sin(t)];
%     Tao_d = [0 0 0]';
%     Tao_d = 0.3*rand(3,1);
    QDD = inv(Mq)*(Tao-Tao_d-Cq_qd*qd-Gq);
    
    qdd_old=QDD;
    qd_old=qd_old+qdd_old*ts;
    q_old=q_old+qd_old*ts;
    
    %%
    % % % % % % % % % % % % % % % learning online  % % % % % % % % % % % % % % % % % % % % % % 
    %%%%    权值W更新    %%%%
    dww = etaw*Fi'*ss';
%     wght = wght + dww';
    
    %%%%    CC  高斯函数均值u更新    %%%%
    for i = 1:3
        for j = 1:8
%             Fi_m(i,j) = 2*Fi(j)*((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))/(v_std(i,j)^2);
            Fi_Mn(j,i+(j-1)*3) = 2*Fi(j)*u_exp(i,j)*((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))/(v_std(i,j)^2);
        end
    end
%     for j = 1:8
%         for i = 1:3
%             Fi_Mn(j,i+(j-1)*3) = Fi_m(i,j);
%         end
%     end
    m_d = etam*Fi_Mn'*wght'*ss;

    %%%%    VV  高斯函数标准差cita更新    %%%%
    for i = 1:3
        for j = 1:8
            Fi_Vs(j,i+(j-1)*3) = 2*Fi(j)*u_exp(i,j)*((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))^2/(v_std(i,j)^3);
        end
    end
    v_d = etav*Fi_Vs'*wght'*ss;
    
    %%%%    R  r_wght更新    %%%%
    for i = 1:3
        for j = 1:8
            Fi_Rw(j,i+(j-1)*3) = -2*Fi(j)*u_exp(i,j)*u_exp_delay(i,j)*((ss(i)+r_w(i,j)*u_exp_delay(i,j))-mns(i,j))/(v_std(i,j)^2);
        end
    end
    r_d = etar*Fi_Rw'*wght'*ss;
    
    %%%%    全部更新 update    %%%%
    wght = wght + dww';
    m_d1 = reshape(m_d,3,8);
    v_d1 = reshape(v_d,3,8);
    r_d1 = reshape(r_d,3,8);
    mns = mns + m_d1;
    v_std = v_std + v_d1;
%     r_w = r_w + r_d1*ts;
    
    
    %%
    % % % % % % % % % % % % % % % plot % % % % % % % % % % % % % % % % % % % % % %    
    Ucmac = [Ucmac uCMAC];
    
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
    
    Un1 = [Un1 uCMAC(1)];
    Urc1 = [Urc1 uRC(1)];
    Un2 = [Un2 uCMAC(2)];
    Urc2 = [Urc2 uRC(2)];
    Un3 = [Un3 uCMAC(3)];
    Urc3 = [Urc3 uRC(3)];
    
    TT=[TT t];
    
    SS=[SS ss];
    
    error_old = error;

end

fprintf('The last error1 is %d;   The RMSE1 is %d\n',error(1),sqrt(sum(E1.^2)/(t/ts)));
fprintf('The last error2 is %d;   The RMSE2 is %d\n',error(2),sqrt(sum(E2.^2)/(t/ts)));
fprintf('The last error3 is %d;   The RMSE3 is %d\n',error(3),sqrt(sum(E3.^2)/(t/ts)));
figure(1)
    subplot(2,1,1);
    hold on;
    plot(TT,Q1,'m--','LineWidth',1.5)
       xlabel('time (s)'); ylabel('angle 1 (rad)');
%     plot(TT,REF1,'--b','LineWidth',1.5)
%        xlabel('time (s)'); ylabel('angle 1 (rad)');
    grid on 
    subplot(2,1,2);
    hold on;
    plot(TT,E1,'m--','LineWidth',1.5)
        xlabel('time (s)'); ylabel('error (rad)');
    grid on
%     subplot(3,1,3);
%     hold on;
%     plot(TT,U1,'m--','LineWidth',1.5)
%         xlabel('time (s)'); ylabel('control effort (rad)');
%     grid on
    
    figure(2)
    subplot(2,1,1);
    hold on;
    plot(TT,Q2,'m--','LineWidth',1.5)
            xlabel('time (s)'); ylabel('angle 2 (rad)');
    grid on 
    subplot(2,1,2);
    hold on;
    plot(TT,E2,'m--','LineWidth',1.5)
            xlabel('time (s)'); ylabel('error (rad)');
    grid on 
%     subplot(3,1,3);
%     hold on;
%     plot(TT,U2,'m--','LineWidth',1.5)
%             xlabel('time (s)'); ylabel('control effort (rad)');
%     grid on
    
    figure(3)
    subplot(2,1,1);
    hold on;
    plot(TT,Q3,'m--','LineWidth',1.5)
            xlabel('time (s)'); ylabel('angle 3 (rad)');
    grid on 
    subplot(2,1,2);
    hold on;
    plot(TT,E3,'m--','LineWidth',1.5)
            xlabel('time (s)'); ylabel('error (rad)');
    grid on
%     subplot(3,1,3);
%     hold on;
%     plot(TT,U3,'m--','LineWidth',1.5)
%             xlabel('time (s)'); ylabel('control effort (rad)');
%     grid on
%     
%     figure(4)
%     subplot(2,1,1);
%     hold on;
%     plot(TT,Un1,'m--','LineWidth',1.5)
%         xlabel('time (s)'); ylabel('Unet (N-m)');
%     grid on;
%     subplot(2,1,2)
%     hold on;
%     plot(TT,Urc1,'m--','LineWidth',1.5)
%         xlabel('time (s)'); ylabel('uRC (N-m)');
%     grid on;
%     
%     figure(5)
%     subplot(2,1,1);
%     hold on;
%     plot(TT,Un2,'m--','LineWidth',1.5)
%         xlabel('time (s)'); ylabel('Unet (N-m)');
%     grid on;
%     subplot(2,1,2)
%     hold on;
%     plot(TT,Urc2,'m--','LineWidth',1.5)
%         xlabel('time (s)'); ylabel('uRC (N-m)');
%     grid on;
%     
%     figure(6)
%     subplot(2,1,1);
%     hold on;
%     plot(TT,Un3,'m--','LineWidth',1.5)
%         xlabel('time (s)'); ylabel('Unet (N-m)');
%     grid on;
%     subplot(2,1,2)
%     hold on;
%     plot(TT,Urc3,'m--','LineWidth',1.5)
%         xlabel('time (s)'); ylabel('uRC (N-m)');
%     grid on;
     
    RMSE = RMSE./(1+Allts/ts);
toc;