function myrobot = mykuka(DH)
    for i=1:6
        L(i) = Link(DH(i,:));
    end
    myrobot = SerialLink(L, 'name', 'kuka');
end