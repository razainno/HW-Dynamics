function a=FK(theta,d,a,alpha)
a=[cos(theta)  -sin(theta)*cosd(alpha) sin(theta)*sind(alpha)  a*cos(theta);
   sin(theta)  cos(theta)*cosd(alpha)  sind(alpha)*cos(theta)  a*sin(theta);
   0                sind(alpha)              cosd(alpha)            d      ;
   0                   0                      0                  1    ];

end
