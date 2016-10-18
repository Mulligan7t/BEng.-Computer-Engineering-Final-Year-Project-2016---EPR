
import bluetooth

target_name = "My Phone"

target_address = "20:82:C0:E9:23:A9"
target_address = "B8:27:EB:FE:EF:75"


nearby_devices = bluetooth.discover_devices()

for bdaddr in nearby_devices:
    if target_name == bluetooth.lookup_name( bdaddr ):
        target_address = bdaddr
        break

if target_address is not None:
    print "found target bluetooth device with address ", target_address
else:
    print "could not find target bluetooth device nearby"