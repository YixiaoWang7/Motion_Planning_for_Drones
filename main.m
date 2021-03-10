clear all
close all
% path points
a=[0.6 0.6 0.55 0.5 0.25 0 -0.25 -0.5 -1 -1 -1 -1 -1 -0.5 -0.25 0 0.25 0.5 1 1 1 1 1 1];
b=[0.6 0.7 0.8 1 1 1 1 1 0.5 0.25 0 -0.25 -0.5 -1 -1 -1 -1 -1 -0.5 -0.25 0 0.25 0.5 1];
c=[0.2 0.3 0.5 0.75 0.75 0.75 0.75 0.75 0.9 0.9 0.9 0.9 0.9 1.34 1.34 1.34 1.34 1.34 1.34 1.34 1.34 1.34 1.34 1.34];
Q=[a;b;c];

n_path_point=length(a)-1; % m
p=4; % order
nc = n_path_point - 3;% number of control points

% generate control points 
[P,nc,U]=generate_control_point1(Q,n_path_point,p,nc);

% parameter interpolation
N=200;% number of interpolated points
u=linspace(0,1,N);
w=ones(1,nc+1);
ROC=zeros(1,N);
RI=zeros(1,N);
Cu=zeros(3,N);
Cuu=zeros(3,N);
Cuu=zeros(3,N);
C1=zeros(3,N);
Cu1=zeros(3,N);
Cuu1=zeros(3,N);
for i=1:N
    R=Rip(U,u(i),w,nc,p);
    C(:,i)=P*R';
    C1(:,i)=deboor(P,U,p,u(i));
    Cu(:,i)=k_deri_of_Curve(U,u(i),nc,p,1,P);
    Cu_norm=sqrt(sum(Cu(:,i).^2));
    C_e(:,i)=Cu(:,i)/Cu_norm;
    Cuu(:,i)=k_deri_of_Curve(U,u(i),nc,p,2,P);
    Cuuu(:,i)=k_deri_of_Curve(U,u(i),nc,p,3,P);
    Cu1(:,i)=deboor_derivative(P,p,u(i),1,U);
    Cuu1(:,i)=deboor_derivative(P,p,u(i),2,U);
    Cuuu1(:,i)=deboor_derivative(P,p,u(i),3,U);
    temp_A=cross(Cu1(:,i),Cuu1(:,i));
    ROC(i)=(Cu1(1,i)^2+Cu1(2,i)^2+Cu1(3,i)^2)^1.5/sqrt(dot(temp_A,temp_A));
end

% plot
plot3(C(1,:),C(2,:),C(3,:),'b-');
hold on
plot3(Q(1,:),Q(2,:),Q(3,:),'r*');
grid on
plot3(P(1,:),P(2,:),P(3,:),'ro');
hold on
axis('equal')
%% trajectory optimization
Vmax=2;
e_max=0.05;
Ts=0.01;
Amax=2;
Jmax=3;
[V_square,favl,flag,f,A,b,Aeq,beq,lb,ub,k]=optimal_speed(Vmax,e_max,ROC,Ts,C1,Cu1,Cuu1,u,Amax); % no jerk limitation
[V_square_final,favl,flag,O,M,B,coe_a]=optimal_final(f,A,b,Aeq,beq,lb,ub,V_square,k,Cu1,Cuu1,Cuuu1,u,Jmax); % with jerk limitation
V=sqrt(V_square_final);
V1=sqrt(V_square);
%% plot
[a,j]=general_aj(V.^2,Cu,Cuu,Cuuu,u,k); %after jerk limitation
[a1,j1]=general_aj(V1.^2,Cu,Cuu,Cuuu,u,k); %before jerk limitation
v = zeros(3,N);
v1 = zeros(3,N);
for i=1:N
    for m=1:3
        v(m,i)=V(i)*Cu1(m,i)/sqrt(dot(Cu1(:,i),Cu1(:,i)));
    end
end
for i=1:N
    for m=1:3
        v1(m,i)=V1(i)*Cu1(m,i)/sqrt(dot(Cu1(:,i),Cu1(:,i)));
    end
end
figure
title('dd')
subplot(3,1,1)
plot(u,v(1,:),'b-');
hold on
plot(u,v1(1,:),'r-');
grid on
xlabel('u')
ylabel('Vx')
legend('after jerk limitation','before jerk limitation')
subplot(3,1,2)
plot(u,v(2,:),'b-');
hold on
plot(u,v1(2,:),'r-');
grid on
xlabel('u')
ylabel('Vy')
legend('after jerk limitation','before jerk limitation')
subplot(3,1,3)
plot(u,v(3,:),'b-');
hold on
plot(u,v1(3,:),'r-');
grid on
xlabel('u')
ylabel('Vz')
legend('after jerk limitation','before jerk limitation')
sgtitle('Vmax=2, Amax=2, Jmax=3')


figure
subplot(3,1,1)
plot(u,a(1,:),'b-');
hold on
plot(u,a1(1,:),'r-');
hold on
xlabel('u')
ylabel('Ax')
legend('after jerk limitation','before jerk limitation')
grid on
subplot(3,1,2)
plot(u,a(2,:),'b-');
hold on
plot(u,a1(2,:),'r-');
legend('after jerk limitation','before jerk limitation')
hold on
xlabel('u')
ylabel('Ay')
grid on
subplot(3,1,3)
plot(u,a(3,:),'b-');
hold on
plot(u,a1(3,:),'r-');
hold on
xlabel('u')
ylabel('Az')
legend('after jerk limitation','before jerk limitation')
grid on
sgtitle('Vmax=2, Amax=2, Jmax=3')

figure
subplot(2,1,1)
plot(u,j(1,:),'b-');
hold on
plot(u,j(2,:),'r-');
hold on
plot(u,j(3,:),'y-');
hold on
xlabel('u')
ylabel('J')
legend('x','y','z')
title('after jerk limitation')
grid on
subplot(2,1,2)
plot(u,j1(1,:),'b-');
hold on
plot(u,j1(2,:),'r-');
hold on
plot(u,j1(3,:),'y-');
hold on
xlabel('u')
ylabel('J')
legend('x','y','z')
title('before jerk limitation')
grid on
sgtitle('Vmax=2, Amax=2, Jmax=3')

%% time interpolation
[Vt,V_vector,t,u_time_final]=V_time(V.^2,u,Ts,P,p,U,0.001);
Vt = [0 Vt 0];
V_vector = [zeros(3,1) V_vector zeros(3,1)];
t = [0 Ts+t t(end)+2*Ts];
u_time_final = [0 u_time_final 1];
m=length(t);
C2 = [];
for i=1:m
    C2(:,i)=deboor (P,U,p,u_time_final(i));
end
figure
subplot(3,1,1)
plot(u_time_final,C2(1,:),'r-');
hold on
plot(u_time_final,C2(2,:),'b-');
hold on
plot(u_time_final,C2(3,:),'y-');
grid on
xlabel('u')
ylabel('d')
legend('x','y','z')
title('u~position')

subplot(3,1,2)
plot(t,C2(1,:),'r-');
hold on
plot(t,C2(2,:),'b-');
hold on
plot(t,C2(3,:),'y-');
grid on
xlabel('t')
ylabel('d')
legend('x','y','z')
title('u~position before replanning')

%% replan the velocity at the beginning and in the end
bi = 3;
ei = length(a)-3;
[rtb,ssb,vsb,asb,jsb] = replan(a(:,bi),v(:,bi),[C(1,bi)-C(1,1);C(2,bi)-C(2,1);C(3,bi)-C(3,1)],Jmax, Ts);
[rte,sse,vse,ase,jse] = replan(-a(:,ei),-v(:,ei),[C(1,ei)-C(1,end);C(2,ei)-C(2,end);C(3,ei)-C(3,end)],Jmax, Ts);
ssb = [ssb(1,:)+C(1,1);ssb(2,:)+C(2,1);ssb(3,:)+C(3,1)];
rte = flip(rte);
rte = abs(rte - max(rte));
sse = flip(sse,2);
sse = [sse(1,:)+C(1,end);sse(2,:)+C(2,end);sse(3,:)+C(3,end)];
vse = -flip(vse,2);
ase = -flip(ase,2);
jse = -flip(jse,2);

[Vt,V_vector,t,u_time_final]=V_time(V(bi:ei).^2,u(bi:ei),Ts,P,p,U,u(bi));
C2 = [];
for i=1:length(u_time_final)
    C2(:,i)=deboor (P,U,p,u_time_final(i));
end
t = [rtb t+rtb(end) t(end)+rtb(end)+rte];
C2 = [ssb C2 sse];
subplot(3,1,3)
plot(t,C2(1,:),'r-');
hold on
plot(t,C2(2,:),'b-');
hold on
plot(t,C2(3,:),'y-');
grid on
xlabel('t')
ylabel('d')
legend('x','y','z')
title('u~position after replanning')








    