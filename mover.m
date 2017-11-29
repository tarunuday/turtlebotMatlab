function mover(pub,msg,lin,ang)
    msg.Linear.X = lin;
    msg.Angular.Z = ang;
    send(pub,msg);
end