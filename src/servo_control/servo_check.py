import serial
import serial.tools.list_ports

# get all port info
ports_list = list(serial.tools.list_ports.comports())

if len(ports_list) == 0:
    print("find nothing")
else:
    print("We find as follows")
    for comport in ports_list:
        print(f"port: {comport.device}, description: {comport.description}")
