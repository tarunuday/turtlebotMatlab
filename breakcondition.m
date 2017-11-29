function safe=breakcondition(t,time)
    globalStopIteration=300;
    globalStopTime=30;
    safe=false;
    if(t>globalStopIteration)
        safe= false;
    elseif(time>globalStopTime)
        safe= false;
    else 
        safe= true;
    end
end
    