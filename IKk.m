function IKk(x,y,z)


a1 =25 ; a2 = 560; b = 0; c1 = 400; c2 = 560; c3 =515; c4 = 90;   % Robot parameters
u = [x;y ;z ];         R = eye(3);             % End effector parameters

C = u - c4*R*[0;0;1];
Cx0 = C(1);         Cy0 = C(2);          Cz0 = C(3);
Nx1 = sqrt(Cx0*Cx0 + Cy0*Cy0 - b*b) - a1;
s1 = sqrt(Nx1*Nx1 + (Cz0-c1)*(Cz0-c1));
s2 = sqrt((Nx1+2*a1)*(Nx1+2*a1) + (Cz0-c1)*(Cz0-c1));
k = sqrt(a2*a2 + c3*c3);

% --------------------------- Positioning Part ---------------------------

theta1 = (atan2(Cy0, Cx0) - atan2(b, Nx1+a1))*(180/pi)
theta2 = (acos((s1*s1 + c2*c2 - k*k)/(2*s1*c2)) + atan2(Nx1, Cz0-c1))*(180/pi)
theta3 = (acos((s1*s1 - c2*c2 - k*k)/(2*c2*k)) - atan2(a2,c3))*(180/pi)

% ----------------------------- Orientation Part -------------------------
Mp = R(1,3) * sin(theta2+theta3) * cos(theta1) + R(2,3) * sin(theta2+theta3) * sin(theta1) + R(3,3) * cos(theta2+theta3)

theta4 = (atan2( R(2,3)*cos(theta1) - R(1,3)*sin(theta1), R(1,3)*cos(theta2+theta3)*cos(theta1) + R(2,3)*cos(theta2+theta3)*sin(theta1) - R(3,3)*sin(theta2+theta3)))*(180/pi)
theta5 = (atan2(sqrt(1-Mp*Mp), Mp))*(180/pi)
theta6 = (atan2(R(1,2)*sin(theta2+theta3)*cos(theta1) + R(2,2)*sin(theta2+theta3)*sin(theta1) + R(3,2)*cos(theta2+theta3) , -R(1,1)*sin(theta2+theta3)*cos(theta1) - R(2,1)*sin(theta2+theta3)*sin(theta1) - R(3,1)*cos(theta2+theta3)))*(180/pi)

save('inverse_kinematics', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6')
end
