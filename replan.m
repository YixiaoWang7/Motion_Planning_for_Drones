function [rt,ss,vs,as,js] = replan(a,v,s,Jmax,Ts)
t = 100;
dt = 1;
pt = t;
while dt > 0.0001
    t = pt - dt;
    A = [t t t;5/2*t^2 3/2*t^2 1/2*t^2; 19/6*t^3 7/6*t^3 1/6*t^3];
    J = [];
    for i=1:3
        J = [J;A\[a(i);v(i);s(i)]];
    end
    if max(abs(J))<Jmax
        pt = t;
    else
        dt = dt/2;
    end
end
t = pt;
A = [t t t;5/2*t^2 3/2*t^2 1/2*t^2; 19/6*t^3 7/6*t^3 1/6*t^3];
for i=1:3
    J = [J;A\[a(i);v(i);s(i)]];
end
rt = 0:Ts:3*t;
if 3*t ~= rt(end)
    rt = [rt 3*t];
end
ss = zeros(3,length(rt));
vs = zeros(3,length(rt));
as = zeros(3,length(rt));
js = zeros(3,length(rt));
for i=1:length(rt)
    if rt(i)>=0 & rt(i) < t
        for j=1:3
            ss(j,i) = J(3*j-2)*rt(i)^3/6;
            vs(j,i) = J(3*j-2)*rt(i)^2/2;
            as(j,i) = J(3*j-2)*rt(i);
            js(j,i) = J(3*j-2);
        end
    else
        if rt(i)>=t & rt(i) < 2*t
            for j=1:3
                ss(j,i) = J(3*j-2)*t^3/6+J(3*j-2)*t^2/2*(rt(i)-t)+J(3*j-2)*t*(rt(i)-t)^2/2+J(3*j-1)*(rt(i)-t)^3/6;
                vs(j,i) = J(3*j-2)*t^2/2+J(3*j-2)*t*(rt(i)-t)+J(3*j-1)*(rt(i)-t)^2/2;
                as(j,i) = J(3*j-2)*t+J(3*j-1)*(rt(i)-t);
                js(j,i) = J(3*j-1);
            end
        else
            if rt(i)>=2*t
                for j=1:3
                    ss(j,i) = J(3*j-2)*t^3/6+J(3*j-2)*t^2/2*t+J(3*j-2)*t*t^2/2+J(3*j-1)*t^3/6+(J(3*j-2)*t^2/2+J(3*j-2)*t*t+J(3*j-1)*t^2/2)*(rt(i)-2*t)+(J(3*j-2)*t+J(3*j-1)*t)*(rt(i)-2*t)^2/2+J(3*j)*(rt(i)-2*t)^3/6;
                    vs(j,i) = J(3*j-2)*t^2/2+J(3*j-2)*t*t+J(3*j-1)*t^2/2+(J(3*j-2)*t+J(3*j-1)*t)*(rt(i)-2*t)+J(3*j)*(rt(i)-2*t)^2/2;
                    as(j,i) = J(3*j-2)*t+J(3*j-1)*t+J(3*j)*(rt(i)-2*t);
                    js(j,i) = J(3*j);
                end
            end
        end
    end
end


end