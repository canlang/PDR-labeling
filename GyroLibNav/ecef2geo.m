function [lat, lon, alt] = ecef2geo(xe, a, e)
%  [lat, lon, alt] = ecef2geo(xe, a, e)
%  Transforms position from the ECEF to geodetic frame
%
%   Input arguments:
%   xe -  Position in ECEF frame [3,1]
%   a  -  Ellipsoid semi-major axis length, m
%   e  -  Ellipsoid eccentricity
%
%   Output arguments:
%   lat, lon, alt - Latitude, Longitude and Altitide in geodetic frame

lon = atan2(xe(2,1),xe(1,1));
alt = 0;
Re = a;
p = sqrt(xe(1,1)^2+xe(2,1)^2);
for i=1:25
    sin_lat = xe(3,1)/((1-e^2)*Re+alt);
    lat = atan((xe(3,1)+e^2*Re*sin_lat)/p);
    Re = a/sqrt(1-e^2*sin(lat)^2);
    alt = p/cos(lat)-Re;
end
end