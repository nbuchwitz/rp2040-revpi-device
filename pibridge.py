import utime
import struct
from machine import UART, Pin

CMD_PING = 0x0002
CMD_SET_FWUPDATE_MODE = 0x0002
CMD_GET_FW_INFO = 0x0003
CMD_WRITE_SERIAL = 0x007
CMD_RESET = 0x0008
CMD_WRITE_MAC = 0x0009
CMD_GET_DEVICE_INFO = 0x000a
CMD_FACTORY_RESET = 0x000b
CMD_DEVICE_INFO = 0x0a

CMD_SET_ADDR = 0x0014
CMD_SET_TERMINATION = 0x0015
CMD_CONFIGURE = 0x0016
CMD_START_DATA_EXCHANGE = 0x0017


def print_hex(payload):
    for c in payload:
        print(hex(c), end=' ')
    print()


class ModGateTelegram:
    MAX_TELEGRAM_DATA_SIZE = 256

    def __init__(self, destination=None, source=None, command=None, sequence=0, length=None, data=None):

        self.__data = bytes()
        self.sequence = sequence

        if source is not None:
            self.source = source

        if destination is not None:
            self.destination = destination

        if command is not None:
            self.command = command

        if data is not None:
            self.data = data

        if length is not None:
            self.length = length
        else:
            self.length = len(self.__data)

    # def __bytes__(self) -> bytes:
    #     return bytes(self.telegram) + self.crc.to_bytes(1, "little")

    def to_bytes(self) -> bytes:
        return bytes(self.telegram) + self.crc.to_bytes(1, "little")

    @staticmethod
    def checksum(payload):
        # bprint(payload)
        # print(payload)
        c = 0

        for el in payload:
            c ^= el

        return c

    # def update(command, data=None):
    #     self.command = command
    #     if data is not None:
    #         self.data = data
    #     else:
    #         self.data = bytes()

    @property
    def data(self):
        return self.__data

    @data.setter
    def data(self, data):
        if not isinstance(data, bytes):
            raise Exception("data must be of type 'bytes'")

        self.__data = data
        self.length = len(data)

    @property
    def telegram(self):
        telegram = bytes()
        telegram += self.destination.to_bytes(1, "little")
        telegram += self.source.to_bytes(1, "little")
        telegram += self.command.to_bytes(2, "little")
        telegram += self.sequence.to_bytes(2, "little")
        telegram += self.length.to_bytes(1, "little")
        telegram += bytes(self.data)

        return telegram

    @property
    def crc(self):
        return self.checksum(self.telegram)

    @staticmethod
    def parse(telegram, ignore_crc=False):
        crc = ModGateTelegram.checksum(telegram[:-1])
        if telegram[-1] != crc and not ignore_crc:
            print(telegram)
            raise Exception(f"invalid checksum {telegram[-1]} {crc}")

        length = struct.unpack('B', telegram[6:7])[0]

        return ModGateTelegram(*struct.unpack(f"<BBHHB", telegram))
        return ModGateTelegram(*struct.unpack(f"BBHHB{length}sx", telegram))


class ModGateRS485:
    BROADCAST_ADDR = 0xff
    ANSWER_OK = 0x4000
    ANSWER_ERROR = 0x8000
    ANSWER = (ANSWER_OK | ANSWER_ERROR)
    ANSWER_FILTER = ~(ANSWER_OK | ANSWER_ERROR)

    def __init__(self, uart):  # , interface: str):
        self.__serial = uart

    def __del__(self):
        # self.__serial.close()
        pass

    def send(self, telegram: ModGateTelegram, timeout=500, ignore_crc=False):
        msg = telegram.to_bytes()
        # print("send: ", end='')
        # print_hex(msg)
        self.__serial.write(msg)

    def receive(self):
        data = bytes()
        while self.__serial.any() > 0:
            data += self.__serial.read(1)

        return data

# module configuration
sn = 12345
module_type = 98 # 98 = DO
hw_revision = 1
sw_major = 2
sw_minor = 0
svn_revision = 42
input_length = 70
output_length = 18
feature_descriptor = 2

# config sniff pins PiBridge
PIN_SNIFF_1A = 13
PIN_SNIFF_1B = 14
PIN_SNIFF_2 = 15

# config UART RS485 PiBridge
UART_NUMBER = 0
UART_BAUDRATE=115200
PIN_UART_TX = 0
PIN_UART_RX = 1

# config module board
PIN_LED_BOARD = 25

# config outputs
PIN_OUTPUT_1 = 2
PIN_OUTPUT_2 = 3
PIN_OUTPUT_3 = 4
PIN_OUTPUT_4 = 5
PIN_OUTPUT_5 = 6
PIN_OUTPUT_6 = 7
PIN_OUTPUT_7 = 8
PIN_OUTPUT_8 = 9

def reset_sniff1():
    global sniff_1a, sniff_1b
    sniff_1a = Pin(PIN_SNIFF_1A, Pin.IN)
    sniff_1b = Pin(PIN_SNIFF_1B, Pin.IN)

def reset_sniff2():
    global sniff_2
    sniff_2 = Pin(PIN_SNIFF_2, Pin.IN)
    sniff_2.irq(irq_sniff2, Pin.IRQ_RISING|Pin.IRQ_FALLING)

def irq_sniff2(pin: Pin):
    if pin.value:
        global rs485_ioprotocol
        rs485_ioprotocol = False
    else:
        reset_sniff1()

sniff_1a: Pin = Pin(PIN_SNIFF_1A, Pin.IN)
sniff_1b: Pin = Pin(PIN_SNIFF_1B, Pin.IN)
sniff_2: Pin = Pin(PIN_SNIFF_2, Pin.IN)
sniff_2.irq(irq_sniff2, Pin.IRQ_RISING|Pin.IRQ_FALLING)

led = Pin(PIN_LED_BOARD, Pin.OUT)
led.off()

OUTPUT_PINS = [PIN_OUTPUT_1, PIN_OUTPUT_2, PIN_OUTPUT_3, PIN_OUTPUT_4, PIN_OUTPUT_5, PIN_OUTPUT_6, PIN_OUTPUT_8]
INPUT_PINS = []


def init_ios(pins, output=False, default_value=None):
    io_list = []

    if output:
        pin_type = Pin.OUT
    else:
        pin_type = Pin.IN

    for pin in pins:
        io = Pin(pin, pin_type)

        if output and default_value is not None:
            io.value(default_value)

        io_list.append(io)

    return io_list

uart_pibridge = UART(UART_NUMBER, baudrate=UART_BAUDRATE, tx=Pin(PIN_UART_TX), rx=Pin(PIN_UART_RX), parity=0, stop=1, bits=8)
bridge = ModGateRS485(uart_pibridge)

address = -1
stage = 0
msg = bytes()
rs485_ioprotocol = True

IOPROTOCOL_MAXDATA_LENGTH = 32
IOPROTOCOL_HEADER_LENGTH = 2

IOPROTOCOL_TYPE1_CMD_DATA = 0
IOPROTOCOL_TYPE1_CMD_CONFIG = 1
IOPROTOCOL_TYPE1_CMD_DATA2 = 2
IOPROTOCOL_TYPE1_CMD_DATA3 = 3
IOPROTOCOL_TYPE1_CMD_DATA4 = 4
IOPROTOCOL_TYPE1_CMD_DATA5 = 5
IOPROTOCOL_TYPE1_CMD_DATA6 = 6
IOPROTOCOL_TYPE1_CMD_DATA7 = 7

MASK_CMD = 0b11100000
MASK_LEN = 0b00011111

REQ_RESPONSE = 0b10000000
IO_HEADER_TYPE_1 = 0x0
IO_HEADER_TYPE_2 = 0b01000000

outputs = init_ios(OUTPUT_PINS, True, False)
inputs = init_ios(INPUT_PINS, False)

configured = False
init_left=False
init_right=False

ts_stage1=0

while True:
    if not rs485_ioprotocol and stage == 0 and sniff_2.value() == True:
        print("stage 0")
        configured = False
        stage = 1
        led.value(False)

        # set sniff 2 to output and drive high in order to keep configuration mode for this particular pibridge side
        sniff_2 = Pin(PIN_SNIFF_2, Pin.OUT)
        sniff_2.value(True)

        ts_stage1 = utime.ticks_ms()
    elif stage == 1:
        delta = (utime.ticks_ms() - ts_stage1)
        print(delta)
        if delta > 200:
            #timeout
            stage=0
            if uart_pibridge.any():
                # clear rx buffer
                uart_pibridge.read()
            msg=bytes()
            reset_sniff2()
            print("timeout")
            continue

        init_stage1=False
        if sniff_1a.value() == False:
            # sniff 1a low means that the module is connected to the right side of RevPi
            print("sniff 1a")
            init_stage1=True
            init_left=False
            init_right=True
        elif sniff_1b.value() == False:
            # sniff 1b low means that the module is connected to the left side of RevPi
            print("sniff 1b")
            init_stage1=True
            init_left=False
            init_right=True
        else:
            stage = 0

        if init_stage1:
            if uart_pibridge.any():
                msg += uart_pibridge.read(1)

                crc = ModGateTelegram.checksum(msg[:-1])

                if msg[-1] == crc:
                    # calculated crc matches last byte of buffer -> telegram complete
                    print_hex(msg)
                    try:
                        # try to parse telegram
                        t = ModGateTelegram.parse(msg)
                    except Exception:
                        # discard telegram
                        msg = bytes()
                        continue

                    if t.command == CMD_GET_DEVICE_INFO:
                        # device info request from RevPi
                        print("device info")
                        msg = bytes()

                        # prepare device info answer (request telegram is used as template)
                        # set response bit in command
                        t.command |= ModGateRS485.ANSWER_OK

                        # add device info data to telegram (type MODGATECOM_IDResp)
                        t.data = sn.to_bytes(4, 'little') \
                            + module_type.to_bytes(2, 'little') \
                            + hw_revision.to_bytes(2, 'little') \
                            + sw_major.to_bytes(2, 'little') \
                            + sw_minor.to_bytes(2, 'little') \
                            + svn_revision.to_bytes(4, 'little') \
                            + input_length.to_bytes(2, 'little') \
                            + output_length.to_bytes(2, 'little') \
                            + feature_descriptor.to_bytes(2, 'little')

                        bridge.send(t)
                    elif t.command == CMD_SET_ADDR:
                        # RevPi wants to assign module address
                        print("set addr", t.destination)

                        # prepare device info answer (request telegram is used as template)
                        # set response bit in command
                        t.command |= ModGateRS485.ANSWER_OK
                        # modules address is set as destionation in telegram
                        address = t.destination
                        # override telegram source with modules address
                        t.source = address
                        # set telegram destination to RevPi
                        t.destination = 0

                        bridge.send(t)

                        stage = 2
                        msg = bytes()
                    else:
                        print("skip: ", end='')
                        print_hex(msg)
                        msg = bytes()
    elif stage == 2:
        if init_left:
            # set sniff 1a to output and drive low, so the next module on the left side can be detected
            # reset to input mode in irq handler when sniff 2 falling edge
            sniff_1a = Pin(PIN_SNIFF_1A, Pin.OUT)
            sniff_1a.value(False)
            print("sniff 1a set L", utime.ticks_us())        
        elif init_right:
            # set sniff 1b to output and drive low, so the next module on the right side can be detected
            # reset to input mode in irq handler when sniff 2 falling edge
            sniff_1b = Pin(PIN_SNIFF_1B, Pin.OUT)
            sniff_1b.value(False)
            print("sniff 1b set L", utime.ticks_us())

        # reset sniff 2 to input after configuration phase is completed
        # if the module detection on one side is completed, sniff 2 will be low -> RevPi module detection finished
        reset_sniff2()
        print("reset sniff 2 to input", utime.ticks_us())
        stage = 0
        configured = True

    else:
        # if not configured:
        #     continue

        if uart_pibridge.any():
            msg += uart_pibridge.read()

            crc = ModGateTelegram.checksum(msg[:-1])

            if msg[-1] == crc:
                if not rs485_ioprotocol:
                    if msg == bytearray([0x7f, 0x00, 0x7f]):
                        # TODO: what is the purpose of this telegram?
                        msg = bytes()
                        continue

                    try:
                        t = ModGateTelegram.parse(msg)
                    except Exception:
                        msg = bytes()
                        continue

                    if t.command == CMD_START_DATA_EXCHANGE:
                        rs485_ioprotocol = True
                        print("start data exchange")
                        led.value(True)

                elif rs485_ioprotocol and msg[0] == address:
                    """
                    cyclic io exchange
                    """
                    iop_cmd = (msg[1] & MASK_CMD) >> 5
                    iop_len = msg[1] & MASK_LEN
                    data = msg[2:2+iop_len]

                    if iop_cmd == IOPROTOCOL_TYPE1_CMD_CONFIG:
                        # received module configuration from RevPi

                        # ack configuration telegram with an empty telegram and reponse bit set
                        response_data = bytearray([])
                        reponse_header = bytearray(
                            [address | REQ_RESPONSE | IO_HEADER_TYPE_1, 0x0])

                    elif iop_cmd == IOPROTOCOL_TYPE1_CMD_DATA:
                        # received module data from RevPi (without PWM)

                        # set outputs with received states
                        # TODO: check for length mismatch (configured vs. present)
                        for n, output in enumerate(outputs):
                            db = int(n/8)
                            value = bool(data[db] & 2**n)
                            output.value(value)

                        # see ioProtocol.h

                        """
                        Format:
                        -----------------------------------------
                        byte 0:   UINT16 Input (first byte)
                        byte 1:   UINT16 Input (second byte)
                        byte 2:   UINT16 Output_Status (first byte)
                        byte 3:   UINT16 Output_Status (second byte)
                        byte 4:   UINT16 Status (first byte)
                        byte 5:   UINT16 Status (second byte)
                        -----------------------------------------
                        """

                        response_data = bytearray(
                            [0x00, 0x00, 0x00, 0x00, 0x0, 0x1])
                        reponse_header = bytearray(
                            [address | REQ_RESPONSE | IO_HEADER_TYPE_1, len(response_data)])

                    elif iop_cmd == IOPROTOCOL_TYPE1_CMD_DATA2:
                        # received module data from RevPi (with PWM)
                        continue
                    elif iop_cmd == IOPROTOCOL_TYPE1_CMD_DATA3:
                        # received module data from RevPi (reset counter values)
                        continue
                    else:
                        # ignore other commands (at least for now)
                        continue

                    # build telegram, append crc and send
                    response = reponse_header + response_data
                    crc = ModGateTelegram.checksum(response)
                    uart_pibridge.write(response + crc.to_bytes(1, "little"))

                msg = bytes()

        if rs485_ioprotocol and len(msg) > IOPROTOCOL_HEADER_LENGTH + IOPROTOCOL_MAXDATA_LENGTH + 1:
            # discard msg buffer if buffer if bigger than maximum message length
            msg = bytes()
