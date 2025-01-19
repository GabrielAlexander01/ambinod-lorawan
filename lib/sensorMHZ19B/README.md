# Driver for the MH-Z19B NDIR CO2 sensor

This driver is heavily inspired from [Erriez MH-Z19B CO2 sensor library for
Arduino](https://github.com/Erriez/ErriezMHZ19B).

It uses an UART serial port to communicate with the sensor. Since the UART
must configured a specific way (9600 8N1), the `mhz19b_init()` takes care of
configuring it.  It will however not start detecting/reading from the sensor.
This must be done. 
