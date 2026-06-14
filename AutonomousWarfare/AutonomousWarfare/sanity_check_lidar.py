import ydlidar
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0")
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000) # קצב הנתונים של TG15

ret = laser.initialize()
if ret:
    ret = laser.turnOn() # כאן המנוע מתחיל להסתובב
    if ret:
        scan = ydlidar.LaserScan()
        if laser.doProcessSimple(scan):
            print(f"Scan received: {scan.points.size()} points")
    laser.turnOff()
