function xe = geo2ecef(lat, lon, alt, a, e)
%  xe = geo2ecef(lat, lon, alt, a, e)
%  Transforms position from the geodetic ot ECEF frame
%
%   Input arguments:
%   lat, lon, alt - Latitude, Longitude and Altitide in geodetic frame
%   a  -  Ellipsoid semi-major axis length, m
%   e  -  Ellipsoid eccentricity
%
%   Output arguments:
%   xe -  Position in ECEF frame [3,1]

Re = a/sqrt(1-e^2*sin(lat)^2);
x = (Re+alt)*cos(lat)*cos(lon);
y = (Re+alt)*cos(lat)*sin(lon);
z = (Re*(1-e^2)+alt)*sin(lat);
xe = [x;y;z];
end