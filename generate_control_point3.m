function [P,n,U] = generate_control_point3 (Q,m,p,n)
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
Q = Q';
M(1,:) = k_deri_of_Nbasis(U,ucontrol(1),n,p,1);
M(2,:) = k_deri_of_Nbasis(U,ucontrol(end),n,p,1);
T = [Q(2,:)-Q(1,:); Q(end,:)-Q(end-1,:)];
W = diag(ones(1,3*(m+1)));
tz = zeros(size(D,1),size(D,2));
D = [D tz tz; tz D tz; tz tz D];
Q = [Q(:,1);Q(:,2);Q(:,3)];
t1 = diag(T(:,1));
t2 = diag(T(:,2));
t3 = diag(T(:,3));
tz = zeros(size(M,1),size(M,2));
M = [t2*M -t1*M tz;t3*M tz -t1*M;tz t3*M -t2*M];
T = zeros(size(M,1),1);
Temp = [M T];
Temp = rref(Temp);
M = Temp(1:rank(Temp),1:end-1);
T = Temp(1:rank(Temp),end);
A = [D'*W*D M';M zeros(size(M,1),size(M,1))];
B = [D'*W*Q;T];
X = A\B;
P = X(1:3*(n+1),:);
P = [P(1:n+1,:) P(n+2:2*(n+1),:) P(2*(n+1)+1:3*(n+1),:)];
P = P';