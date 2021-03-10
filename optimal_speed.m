function [V_square,favl,flag,f,A,b,Aeq,beq,lb,ub,k]=optimal_speed(Vmax,e_max,ROC,Ts,C,Cu,Cuu,u,Amax)
n=length(ROC);
lb=zeros(n,1);
ub=zeros(n,1);
for i=1:n
    k(i)=sqrt(sum(Cu(:,i).*Cu(:,i)));
end
for i=1:n
    ub(i,1)=min(Vmax^2,8*ROC(i)*e_max/Ts^2);
end

% acceleration limitation
A=zeros(3*(n-2),n);
for i=2:n-1
    for j=1:3
        A_xyz=[-Cu(j,i)/(2*(u(i+1)-u(i-1))*k(i-1)^2) Cuu(j,i)/k(i)^2 Cu(j,i)/(2*(u(i+1)-u(i-1))*k(i+1)^2)];
        A(3*(i-2)+j,i-1:i+1)=A_xyz;
    end
end

% linear programming
A=[A;-A];
b=Amax*ones(6*(n-2),1);
Aeq=zeros(2,n);
beq=zeros(2,1);
Aeq(1,1)=1;
Aeq(2,n)=1;
f=-ones(1,n);
[V_square,favl,flag]=linprog(f,A,b,Aeq,beq,lb,ub);

