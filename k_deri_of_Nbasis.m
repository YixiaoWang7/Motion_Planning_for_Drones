function [deri_N]=k_deri_of_Nbasis(U,u,n,p,k) %NΪ��������UΪ�ڵ�������pΪ������kΪ�󵼵Ĵ�����nΪ���Ƶ�������һ
deri_N=Nbasis(U,u,n,p-k);
for j=1:k
    for i=1:n+1
        a=(p-k+j)/(U(i+p-k+j)-U(i));
        b=(p-k+j)/(U(i+p-k+j+1)-U(i+1));
        if U(i+p-k+j)-U(i)==0
            a=0;
        end
        if U(i+p-k+j+1)-U(i+1)==0
            b=0;
        end
        deri_N(i)=a*deri_N(i)-b*deri_N(i+1);
    end
end
deri_N=deri_N(1:n+1);

    
    