function [P,n,U] = generate_control_point1 (Q,m,p,n)
ucontrol=linspace(0,1,m+1);
for i=1:m
    d(i)=abs(sum((Q(:,i)-Q(:,i+1)).*(Q(:,i)-Q(:,i+1)))); % the parameterized value proportional to the length
end
l=sum(d);
ucontrol(1)=0;
for i=2:m+1
    ucontrol(i)=ucontrol(i-1)+d(i-1)/l;
end
c=(m+1)/(n-p+1);
U=zeros(1,n+p+2);
U(n+2:n+p+2)=ones(1,p+1); 
for j=1:n-p
    i=floor(j*c);
    afa=j*c-i;
    U(p+j+1)=(1-afa)*ucontrol(i)+afa*ucontrol(i+1);
end

for i=1:m+1
    N=Nbasis(U,ucontrol(i),n,p);
    N=N(1:n+1);
    D(i,:)=N;
end
P=(D'*D)\(D'*Q');
P=P';


% Q = Q';
% for i=1:m-1
%     N=Nbasis(U,ucontrol(i+1),n,p);
%     R(i,:)=Q(i+1,:)-Q(1,:)*N(1)-Q(end,:)*N(n+1);
% end
% for i=1:m-1
%     N=Nbasis(U,ucontrol(i+1),n,p);
%     D(i,:)=N(2:n);
% end
% P=(D'*D)^-1*(D'*R);
% P=[Q(1,:);P;Q(end,:)];
% P=P';