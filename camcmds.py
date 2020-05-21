import serial, time

class axsys_control():

    def __init__(self, camera_serial_port, camera_type=1):
        self.haltReads = False

        byte_size = serial.EIGHTBITS
        parity_bits = serial.PARITY_NONE

        if (camera_type is 3):
            stop_bits = serial.STOPBITS_TWO
            self.line_ending = "\x0D"
        elif (camera_type is 2):
            print("Unimplemented camera type")
            exit()
        elif (camera_type is 1):
            print("")
            stop_bits = serial.STOPBITS_ONE
            self.line_ending = "\x0D"

        # Try to open serial port
        try:
            self.serial_enabled = True
            self.ser = serial.Serial(
                camera_serial_port, 
                baudrate=57600, 
                timeout=0, 
                bytesize=byte_size, 
                parity=parity_bits, 
                stopbits=stop_bits
            )

        except:
            print("Could not enable serial port on {}".format(camera_serial_port))
            self.serial_enabled = False

    def agc(self, value):
        cmd = "AGC:{:d}".format(value)
        self.camCmd(cmd)

    def moc(self, value):
        cmd = "MOC:{:d}".format(value)
        self.camCmd(cmd)

    def gain(self, value):
        cmd = "MGC:{:d}".format(value)
        self.camCmd(cmd)

    def senstog(self, value):
        cmd = "SENSTOG:{:d}".format(value)
        self.camCmd(cmd)

    def focuspos(self, value):
        cmd = "FOCUSPOS:{:d}".format(value)
        self.camCmd(cmd)

    def readPort(self):
        
        if self.serial_enabled:
            if self.haltReads is not True:
                feedback = self.ser.read(64)
                if(feedback is not b''):
                    return feedback
                else:
                    return None

    def camCmd(self, cmd):
        if self.serial_enabled:
            cmd = cmd + self.line_ending
            print(cmd)
            self.ser.write(cmd.encode('ascii'))


    def camCmdWithResponse(self, cmd):
        
        if self.serial_enabled:
            self.haltReads = True

            #timeout = time.monotonic() + 0.5
            #while timeout > time.monotonic():
            self.ser.reset_input_buffer()

            og_cmd = cmd
            cmd = cmd + self.line_ending
            print(cmd)

            self.ser.write(cmd.encode('ascii'))

            result = ''
            timeout = time.monotonic() + 0.1
            while timeout > time.monotonic():
                input = self.ser.read(64)
                if(input is not b''):
                    result += input.decode('latin-1')

            self.haltReads = False

            if result != b'':
                result_str = str(result)
                print(result_str)
                result_str = result_str.replace("\r", '')
                result_str = result_str.replace("\n", '')
                result_str = result_str.replace(str(og_cmd), '')
                result_str = result_str.replace('>', '')
                return result_str
            else:
                return None
            