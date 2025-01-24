function dislivello = crea_dislivello(len,x)
    dislivello=zeros(1,len);
    start=ceil(len*x);
    len_discesa=len;
    i=start;
    while (i<len_discesa+start && i<len)
        dislivello(i)=-x*5.71*pi/180;
        i=i+1;
    end
end