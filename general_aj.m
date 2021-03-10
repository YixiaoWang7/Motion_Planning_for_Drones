function [A,J]=general_aj(V_square_final,Cu,Cuu,Cuuu,u,k)
n=length(u);
q=V_square_final'./(k.^2);
A=zeros(3,n);
J=zeros(3,n);
for i=2:n-1
    for j=1:3
        A(j,i)=-Cu(j,i)/(2*(u(i+1)-u(i-1)))*q(i-1)+Cuu(j,i)*q(i)+Cu(j,i)/(2*(u(i+1)-u(i-1)))*q(i+1);
    end
end
du=u(2)-u(1);
coe_a=Cu/(2*du^2)-3*Cuu/(4*du);
coe_b=Cuuu-Cu/du^2;
coe_c=Cu/(2*du^2)+3*Cuu/(4*du);
for i=2:n-1
    for j=1:3
        J(j,i)=(coe_a(j,i)*q(i-1)+coe_b(j,i)*q(i)+coe_c(j,i)*q(i+1))*sqrt(q(i));
    end
end

