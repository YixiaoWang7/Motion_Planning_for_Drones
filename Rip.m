function [R] = Rip(U,u,w,n,p)
N=Nbasis(U,u,n,p);
N=N(1:n+1);
for i=1:n+1
    R(i) = N(i)*w(i)/(N*w');
end