function [myrobot] = mypuma560(DH)
    for i=1:6
        L(i) = Link(DH(i,:));
    end
    myrobot = SerialLink(L, 'name', 'puma560');
end