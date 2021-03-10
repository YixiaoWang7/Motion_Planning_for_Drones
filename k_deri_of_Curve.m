function [C]=k_deri_of_Curve(U,u,n,p,k,P) %N为基函数，U为节点向量，p为阶数，k为求导的次数，n为控制点数量减一,P为控制点集
deri=k_deri_of_Nbasis(U,u,n,p,k);
C=P*deri';