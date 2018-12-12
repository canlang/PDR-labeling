function xn = geo2ned(lat, lon, alt, lat0, lon0, alt0, a, e)
%  xn = geo2ned(lat, lon, alt, lat0, lon0, alt0, a, e)
%  Transforms position from the geodetic to NED frame
%
%   Input arguments:
%   lat, lon, alt - Latitude, Longitude and Altitide in geodetic frame
%   lat0, lon0, alt0 - Latitude, Longitude and Altitide of NED frame origin
%   a  -  Ellipsoid semi-major axis length, m
%   e  -  Ellipsoid eccentricity
%
%   Output arguments:
%   xn -  Position in NED frame [3,1]

xe  = geo2ecef(lat, lon, alt, a, e);
xe0  = geo2ecef(lat0, lon0, alt0, a, e);
Cen = [...
    -sin(lat0)*cos(lon0) -sin(lat0)*sin(lon0)  cos(lat0)
              -sin(lon0)            cos(lon0)          0
    -cos(lat0)*cos(lon0) -cos(lat0)*sin(lon0) -sin(lat0)
    ];
xn = Cen*(xe-xe0);
end