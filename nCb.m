function XYZ = nCb(XYZ_b)

    Rpsi = [cos(psi), -sin(psi), 0;
            sin(psi), cos(psi), 0;
            0,0,1];
    
    Rtheta = [cos(theta), 0, sin(theta);
              0,1,0;
              -sin(theta), 0, cos(theta)];
    
    Rphi = [1,0,0;
            0,cos(phi),-sin(phi);
            0,sin(phi),cos(phi)];
    
    bCn = Rpsi*Rtheta*Rphi;
    
    XYZ = bCn.'*XYZ_b;

end