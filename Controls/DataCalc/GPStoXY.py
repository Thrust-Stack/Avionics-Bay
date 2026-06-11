import math

lat = InsertLatitudeHere
lon = InsertLongitudeHere

Where = "TCC"

# TCC's Latitude & Longitude
TCClat = 36.51092
TCClon = 120.06880
# FAR Latitude & Longitude
FARlat = 35.34761
FARlon = 117.80902

if Where == "TCC":
    sitelat = TCClat
    sitelon = TCClon
elif Where == "FAR":
    sitelat = FARlat
    sitelon = FARlon
else:
    sitelat = 0
    sitelon = 0 

def latlon_to_xy(lat, lon, sitelat, sitelon):
    R = 6371000  # Earth radius in meters

    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    sitelat_rad = math.radians(sitelat)
    sitelon_rad = math.radians(sitelon)

    dlat = lat_rad - sitelat_rad
    dlon = lon_rad - sitelon_rad

    GPSx = R * dlon * math.cos(sitelat_rad)
    GPSy = R * dlat

    return GPSx, GPSy