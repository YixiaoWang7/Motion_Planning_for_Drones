function [C]=k_deri_of_Curve2(U,u,n,p,k,P) %N为基函数，U为节点向量，p为阶数，k为求导的次数，n为控制点数量减一,P为控制点集
for i=1:k
    P=general_k_point(P,U,p,k);
end
U(1:k)=[];
U(end-k+1:end)=[];
deri=Nbasis(U,u,n-k,p-k);
deri=deri(1:n+1-k);
C=P*deri';