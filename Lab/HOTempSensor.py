import time
import smbus

i2c_ch = 1
i2c_address = 0x48

reg_temp = 0x00

def read_temp():
    regVal = bus.read_i2c_block_data(i2c_address, reg_temp, 2)
    print("1st byte", bin(regVal[0]))
    print("2nd byte", bin(regVal[1]))
    
    temp_combined = (regVal[0] << 4) | (regVal[1] >> 4))
    print("Combined",bin(temp_combined))
    
    temp_degC = temp_combined * 400
    
    return temp_degC

bus = smbus.SMBus(i2c_ch)
while True:
    temperature = read_temp()
    print('The room temperature is ', temperature, 'DegC')
    time.sleep(1)
