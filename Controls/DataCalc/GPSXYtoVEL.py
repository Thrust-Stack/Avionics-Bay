GPSx0 = 0 #Last GPS x position
GPSy0 = 0 #Last GPS y position
GPSz0 = 0 #Last GPS z position
GPSt0 = 0 #Last GPS timestamp

GPSx1 = 0 #Current GPS x position
GPSy1 = 0 #Current GPS y position
GPSz1 = 0 #Current GPS z position
GPSt1 = 0 #Current GPS timestamp

def GPS_to_velocity(GPSx0, GPSy0, GPSz0, GPSt0, GPSx1, GPSy1, GPSz1, GPSt1):
    # Calculate the change in position and time
    delta_x = GPSx1 - GPSx0
    delta_y = GPSy1 - GPSy0
    delta_z = GPSz1 - GPSz0
    delta_t = GPSt1 - GPSt0

    # Avoid division by zero
    if delta_t == 0:
        return 0, 0, 0

    # Calculate velocity components
    velocity_x = delta_x / delta_t
    velocity_y = delta_y / delta_t
    velocity_z = delta_z / delta_t
    total_velocity = (velocity_x**2 + velocity_y**2 + velocity_z**2)**0.5

    return velocity_x, velocity_y, velocity_z, total_velocity
