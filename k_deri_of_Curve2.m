function [C]=k_deri_of_Curve2(U,u,n,p,k,P) %NΪ��������UΪ�ڵ�������pΪ������kΪ�󵼵Ĵ�����nΪ���Ƶ�������һ,PΪ���Ƶ㼯
for i=1:k
    P=general_k_point(P,U,p,k);
end
U(1:k)=[];
U(end-k+1:end)=[];
deri=Nbasis(U,u,n-k,p-k);
deri=deri(1:n+1-k);
C=P*deri';