function res = search(x,y,z,listx,listy)
    
    posx=find(listx >= x, 1);
    posy=sum(y>=listy);
    % pos=ceil(length(listx)/2);
    % posx=pos;
    % tmp=listx;
    % while(length(tmp)>1)
    %     if x==tmp(pos)
    %         break
    %     elseif(x>=tmp(pos))
    %         tmp=tmp(pos+1:end);
    %         pos=ceil(length(tmp)/2);
    %         posx=posx+pos;
    %     elseif x<=tmp(pos)
    %         tmp=tmp(1:pos);
    %         pos=ceil(length(tmp)/2);
    %         posx=ceil(posx-length(tmp)/2);
    %     end
    % end
    % 
    % pos=ceil(length(listy)/2);
    % tmp=listy;
    % posy=pos;
    % while(length(tmp)>1)
    %     if y==tmp(pos)
    %         break
    %     elseif(y>=tmp(pos))
    %         tmp=tmp(pos+1:end);
    %         pos=ceil(length(tmp)/2);
    %         posy=posy+pos;
    %     elseif y<=tmp(pos)
    %         tmp=tmp(1:pos);
    %         pos=ceil(length(tmp)/2);
    %         posy=ceil(posy-length(tmp)/2);
    %     end
    % end
    if posx>1 && posy>1
    res=(abs(z(posy,posx)*(listy(posy-1)-y)/(listy(posy)-listy(posy-1)))+abs(z(posy-1,posx)*(listy(posy)-y)/(listy(posy)-listy(posy-1)))+...
        abs(z(posy,posx)*(listx(posx-1)-x)/(listx(posx)-listx(posx-1)))+abs(z(posy,posx-1)*(listx(posx)-x)/(listx(posx)-listx(posx-1))))/2;
    else
        res=z(posy,posx);
    end
end