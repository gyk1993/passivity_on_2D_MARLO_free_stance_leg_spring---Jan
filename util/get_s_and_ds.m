function [ s ds ] = get_s_and_ds( q,dq,theta_begin,theta_end )

s=get_s(q,dq,theta_begin,theta_end);
ds=get_ds(q,dq,theta_begin,theta_end);

if s>1
    s=1;
%     ds=0;
end
if s<0
    s=0;
%     ds=0;
end

end

