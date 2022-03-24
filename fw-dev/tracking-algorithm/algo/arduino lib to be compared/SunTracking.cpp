/*
Sun Tracking Library for Arduino
Based on Sun Equations from the book
Jean Meeus - "Astronomical Algorithms"
Created by Alberto Perro, 2015.
Released into public domain.
*/
#include <Arduino.h>
#include <SunTracking.h>

void SunTracker::Julian(int hh,int mm,int ss,int DD, int MM, int YY)
{
  float UT = (hh+mm/60.+ss/3600.)/24.;
  float JD = 367.*YY-int(7*(YY+int((MM+9)/12))/4)+int(275*MM/9)+DD+1721013.5+UT;
  _JCE = (JD-2451545.)/36525.;
  Serial.println("JD "+String(JD)+"\n");
  //Serial.println("JCE "+String(_JCE)+"\n");
}
void SunTracker::SunCoordinates(int hh,int mm,int ss,int DD, int MM, int YY)
{
  Serial.println("HH "+String(hh));
  Serial.println("mm "+String(mm));
  Serial.println("ss "+String(ss));
  Serial.println("dd "+String(DD));
  Serial.println("MM "+String(MM));
  Serial.println("YY "+String(YY));
  Julian(hh,mm,ss,DD,MM,YY);
  _L0 = (280.46645+36000.72983*_JCE+0.0003032*_JCE*_JCE);
  _L0 = fmod(_L0,360.);
  _M  = 357.52910+35999.05030*_JCE+0.0001559*_JCE*_JCE-0.00000048*_JCE*_JCE*_JCE;
  _e  = 0.016708617-0.000042037*_JCE-0.0000001236*_JCE*_JCE;
  _Center  = (1.914602-0.004187*_JCE-0.000014*_JCE*_JCE)*sin(_M*_PI/180.)+(0.01993-0.000101*_JCE)*sin(2.*_M*_PI/180.)+0.000289*sin(3.*_M*_PI/180.);
  _TLon = _L0+_Center;
  _TM = _M+_Center;
  _R = 1.000001018*(1-_e*_e)/(1+_e*cos(_TM*_PI/180.));
  _Omega = 125.04-1934.136*_JCE;
  _ALon = _TLon-0.00569-0.00478*_Omega;
  _eps = 23.43929-0.01300417*_JCE-1.63889e-7*_JCE*_JCE+5.036111e-7*_JCE*_JCE*_JCE;
  _Dec = 180.*asin(sin(_eps*_PI/180.)*sin(_TLon*_PI/180.))/_PI;
  _RA  = 180.*atan2(cos(_eps*_PI/180.)*sin(_TLon*_PI/180.),cos(_TLon*_PI/180.))/_PI;
}
float SunTracker::getDeclination(int hh,int mm,int ss,int DD, int MM, int YY)
{
  SunCoordinates(hh,mm,ss,DD,MM,YY);
  return _Dec;
}
float SunTracker::getRightAscension(int hh,int mm,int ss,int DD, int MM, int YY)
{
  SunCoordinates(hh,mm,ss,DD,MM,YY);
  return _RA;
}
float SunTracker::getAzimuth(int hh,int mm,int ss,int DD, int MM, int YY)
{
  SunCoordinates(hh,mm,ss,DD,MM,YY);
  float q0 = 280.46061837+360.98564736629*_JCE*36525.+ 0.000387933*_JCE*_JCE-_JCE*_JCE*_JCE/38710000;
  _HA = q0+_Lon-_RA;
  float Az=sin(_HA*_PI/180.)/(cos(_HA*_PI/180.)*sin(_Lat*_PI/180.)-tan(_Dec*_PI/180.)*cos(_Lat*_PI/180.));
  Az = 180./_PI*atan(Az);
  return Az;
}
float SunTracker::getElevation(int hh, int mm, int ss, int DD, int MM, int YY)
{
  SunCoordinates(hh,mm,ss,DD,MM,YY);
  float q0 = 280.46061837+360.98564736629*_JCE*36525.+ 0.000387933*_JCE*_JCE-_JCE*_JCE*_JCE/38710000;
  _HA = q0+_Lon-_RA;
  float El = 180./_PI*asin(sin(_Lat*_PI/180.)*sin(_Dec*_PI/180.)+cos(_Lat*_PI/180.)*cos(_Dec*_PI/180.)*cos(_HA*_PI/180.));
  return El;
}
void SunTracker::setLatLon(float lat, float lon)
{
  _Lat = lat;
  _Lon = lon;
}
