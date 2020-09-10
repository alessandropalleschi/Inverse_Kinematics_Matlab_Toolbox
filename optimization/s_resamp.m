function s_res = s_resamp(s,t_opt,b,a,t,indice)
%S_RESAMP Summary of this function goes here
%   Detailed explanation goes here
i=2;
if(t>t_opt(end))
    disp('Error')
    return;
end

i = max(size(t_opt( 1:find( t_opt > t, 1 ) )));

s_dot = gradient(s, t_opt);

if indice ==1
%% st using initial and final position
if isempty(t_opt( 1:find( t_opt > t, 1 ) ))
    i=max(size(s));
% else
end
    if t<0
        i=2;
    end

    c1 = a(i-1);
    c2 = (s(i)-s(i-1))/(t_opt(i)-t_opt(i-1)) - a(i-1)*(t_opt(i)-t_opt(i-1))/2;
    c3 = s(i-1);

    s_res = 0.5*c1*(t-t_opt(i-1))^2+c2*(t-t_opt(i-1))+c3;
    if(s_res>1)
        disp('WTF')
    end
% end
% if t<0
%     i=2;
% end
elseif indice == 2
%% Linear interp
if isempty(t_opt( 1:find( t_opt > t, 1 ) ))
    i=max(size(s));
% else
end   
    if t<0
        i=2;
    end

    c2 = (s(i)-s(i-1))/(t_opt(i)-t_opt(i-1));
    c3 = s(i-1);

    s_res = c2*(t-t_opt(i-1))+c3;
    if(s_res>1)
        disp('WTF')
    end
% end
% if t<0
%     i=2;
% end

    elseif indice == 3
%% NEW 
% s(t) = c1*t^2+c2*t+c3

% c1 = a(i-1);
% c2 = (s(i)-s(i-1))/t_opt(i) - a(i-1)*t_opt(i)/2;
% c3 = s(i-1);
% 
% s_res = 0.5*c1*(t-t_opt(i-1))^2+c2*(t-t_opt(i-1))+c3;
%% forward integration
if isempty(t_opt( 1:find( t_opt > t, 1 ) ))
    i=max(size(s));
end
if t<0
    i=2;
end
s_res = s(i-1)+b(i-1)^(1/2)*(t-t_opt(i-1))+1/2*a(i-1)*(t-t_opt(i-1))^2;
    end
end

