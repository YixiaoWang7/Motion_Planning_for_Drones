function [V_square_final,favl,flag,A,q_first,b,coe_a]=optimal_final(f,A,b,Aeq,beq,lb,ub,V_square,k,Cu,Cuu,Cuuu,u,Jmax)
q_first=V_square'./(k.^2);
du=u(2)-u(1);
coe_a=Cu/(2*du^2)-3*Cuu/(4*du);
coe_b=Cuuu-Cu/du^2;
coe_c=Cu/(2*du^2)+3*Cuu/(4*du);
n=length(V_square);
B=zeros(6*n-12,n);

% acceleration and jerk limitation
for i=2:n-1
    for j=1:3
        B_xyz=[sqrt(q_first(i))*coe_a(j,i)/k(i-1)^2 sqrt(q_first(i))*coe_b(j,i)/k(i)^2+Jmax*0.5/(q_first(i)*k(i)^2) sqrt(q_first(i))*coe_c(j,i)/k(i+1)^2];
        B(3*(i-2)+j,i-1:i+1)=B_xyz;
    end
end
B(3*n-5:6*n-12,:)=-B(1:3*n-6,:);
for i=2:n-1
    for j=1:3
        B(3*(i-2)+j+3*(n-2),i)=B(3*(i-2)+j+3*(n-2),i)+Jmax/(k(i)^2*q_first(i));
    end
end

% jerk limitation at the beginning and in the end
coe_d=(Cu(:,3).^2*sqrt(q_first(2)))./(8*Cu(:,2)*k(3)^2*du^2);
coe_e=(Cu(:,n-2).^2*sqrt(q_first(n-1)))./(8*Cu(:,n-1)*k(n-2)^2*du^2);
C=zeros(12,n);
for i=1:3
    if Cu(i,n-1)==0
        coe_e(i)=10;
    end
end
for i=1:3
    C(i,2:3)=[Jmax/(2*q_first(2)*k(2)^2) coe_d(i)];
end
for i=1:3
    C(i+3,n-2:n-1)=[coe_e(i) Jmax/(2*q_first(n-1)*k(n-1)^2)];
end
for i=1:3
    C(i+6,2:3)=[Jmax/(2*q_first(2)*k(2)^2) -coe_d(i)];
end
for i=1:3
    C(i+9,n-2:n-1)=[-coe_e(i) Jmax/(2*q_first(n-1)*k(n-1)^2)];
end

% linear programming
A=[A;B;C];
b=[b;ones(6*n,1)*Jmax*1.5];
[V_square_final,favl,flag]=linprog(f,A,b,Aeq,beq,lb,ub);
    