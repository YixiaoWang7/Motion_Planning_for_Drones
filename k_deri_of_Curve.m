function [C]=k_deri_of_Curve(U,u,n,p,k,P) %NΪ��������UΪ�ڵ�������pΪ������kΪ�󵼵Ĵ�����nΪ���Ƶ�������һ,PΪ���Ƶ㼯
deri=k_deri_of_Nbasis(U,u,n,p,k);
C=P*deri';