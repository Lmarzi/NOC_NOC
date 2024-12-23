function dislivello = crea_dislivello(len,len_discesa)
    i=1;
    dislivello=zeros(len,1);
    while i<len
        if rand>0.9
            start=i;
            while(i-start<len_discesa && i<len)
            dislivello_this=-10*rand*2*pi/180;
            start_2=i;
            while(i-start_2<len_discesa/5 && i<len)
                dislivello(i)=dislivello_this;
                i=i+1;
            end
            end
        end
    i=i+1;
    end
end