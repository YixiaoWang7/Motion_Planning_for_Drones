function [V,V_vector,t,u_time_final]=V_time(V_point,u,Ts,P,p,U,ui)
du=u(2)-u(1);
u_time=ui;
N=length(u);
l=1;
i=1;
flag1=1;
flag2=1;
while(flag2)
   while(flag1)
       if u_time>=u(i)&&u_time<u(i+1)
           V(l)=(V_point(i+1)-V_point(i))/du*(u_time-u(i))+V_point(i);
           flag1=0;
       else
           if u_time>=u(end)
               V(l)=V_point(end);
               flag1=0;
               flag2=0;
           else
               i=i+1;
           end
       end
   end
   if flag2 == 0
       break
   end
   Cu=deboor_derivative(P,p,u_time,1,U);
   Cuu=deboor_derivative(P,p,u_time,2,U);
   Cu_norm=sqrt(sum(Cu.^2));
   Cuu_norm=sqrt(sum(Cuu.^2));
   V_vector(1,l)=V(l)*Cu(1)/Cu_norm;
   V_vector(2,l)=V(l)*Cu(2)/Cu_norm;
   V_vector(3,l)=V(l)*Cu(3)/Cu_norm;
   u_time=u_time+V(l)/Cu_norm*Ts-V(l)^2/Cu_norm^3*Cuu_norm*Ts^2/2;
   u_time_final(l)=u_time;
   t(l)=(l-1)*Ts;
   l=l+1; 
   flag1=1;
   if u_time>=u(end)-0.001
       break
   end
end



    
