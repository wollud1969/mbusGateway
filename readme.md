## Disable the serial console, disable bluetooth and disable the hciuart service:

https://www.raspberrypi.org/documentation/configuration/uart.md
https://www.raspberrypi.org/forums/viewtopic.php?t=178071


### Disable hciuart and other bluetooth related services:
* `systemctl disable hciuart`
* `systemctl disable bluealsa`
* `systemctl disable bluetooth`

### Disable serial console:
* Check `/boot/cmdline.txt`
* Comment line `console=serial0,115200` or `console=ttyAMA0,115200` or similar

### Disable bluetooth
* Add `dtoverlay=pi3-disable-bt` to `/boot/config.txt`



## Serial port configuration in Python:

https://pythonhosted.org/pyserial/shortintro.html#opening-serial-ports

Consider a timeout!

