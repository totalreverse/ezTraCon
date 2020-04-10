#!/usr/bin/python3


import usb.core
import usb.util
import time

MESG_UNASSIGN_CHANNEL_ID       = 0x41
MESG_OPEN_CHANNEL_ID           = 0x4B
MESG_CLOSE_CHANNEL_ID          = 0x4C
MESG_BROADCAST_DATA_ID         = 0x4E
MESG_ACKNOWLEDGED_DATA_ID      = 0x4F
MESG_OPEN_RX_SCAN_ID           = 0x5B
MESG_EXT_BROADCAST_DATA_ID     = 0x5D
MESG_EXT_ACKNOWLEDGED_DATA_ID  = 0x5E
#
MESG_ID_LIST_ADD_ID            = 0x59
#
MESG_BURST_DATA_ID             = 0x50
MESG_EXT_BURST_DATA_ID         = 0x5F
#
MESG_SYSTEM_RESET_ID           = 0x4A
# 0xC7 may permanently reflash your USB
MESG_SET_USB_INFO_ID           = 0xC7


MESG_VERSION_ID                = 0x3E
MESG_CHANNEL_ID_ID             = 0x51
MESG_CHANNEL_STATUS_ID         = 0x52
MESG_CAPABILITIES_ID           = 0x54
MESG_STACKLIMIT_ID             = 0x55
MESG_GET_SERIAL_NUM_ID         = 0x61
MESG_FLASH_PROTECTION_CHECK_ID = 0xA7
MESG_GET_GRMN_ESN_ID           = 0xC6

valid_request_ids = [
     MESG_VERSION_ID,
     MESG_CHANNEL_ID_ID,
     MESG_CHANNEL_STATUS_ID,
     MESG_CAPABILITIES_ID,
     MESG_STACKLIMIT_ID,
     MESG_GET_SERIAL_NUM_ID,
     MESG_FLASH_PROTECTION_CHECK_ID,
     MESG_GET_GRMN_ESN_ID ]


ctrlReset               = "4A 00"      # Reset
ctrlResetJunkChannelOK  = "4A 07"      # Reset with valid 'illegal channel'
ctrlResetJunkChannelErr = "4A 08"      # Reset with invalid
ctrlResetNoChannel      = "4A"         # Reset

requestAntVersion   = "4D 00 3e"   # request ant version
requestChannelId    = "4D 00 51"   # request channel id
requestChannelStatus= "4D 00 52"   # request channel status
requestCapabilities = "4D 00 54"   # request capabilities
requestStackLimit   = "4D 00 55"   # request stack limit
requestSerial       = "4D 00 61"   # Request 0x61 - serial number
requestFlashProtect = "4D 00 A7"   #
requestGarminEsd    = "4D 00 C6"   # MESG_GET_GRMN_ESN_ID: i.e. 38,31,30,32,33 => "81023" or a0,a1,a2,a3,a4

configTxPowerFF     = "47 00 FF"   # Tx Power
configAntNetworkKey = "46 00 B9 A5 21 FB BD 72 C3 45"    # set ANT+ standard network key for NET #0

conigLibE0          = "6E 00 E0"   # Lib Config : (all extensions allowed) - device command (channel not relevant)
conigLib20          = "6E 00 20"   # Lib Config : (all extensions allowed) - device command (channel not relevant)
conigLib40          = "6E 00 40"   # Lib Config : (all extensions allowed) - device command (channel not relevant)
conigLib80          = "6E 00 80"   # Lib Config : (all extensions allowed) - device command (channel not relevant)
conigLib66ON        = "66 00 01"   # (2nd byte != 0) Libconfig |= 80  => 0x60 => 0xe0
conigLib66OFF       = "66 00 00"   # (2nd byte == 0) Libconfig &= ~80 switch 0x80 to zero => 0xe0 => 0x60

requestCapabilitiesJunkNotOK4 = "4D ff 54"   # request capabilities with illegal "channel"
requestCapabilitiesJunkOK     = "4D e7 54"   # request capabilities with illegal "channel"
requestCapabilitiesJunkOK1    = "4D 07 54"   # request capabilities with illegal "channel"
requestCapabilitiesJunkNotOK1 = "4D 08 54"   # request capabilities with illegal "channel"
requestCapabilitiesJunkNotOK2 = "4D f7 54"   # request capabilities with illegal "channel"
requestCapabilitiesJunkNotOK3 = "4D e8 54"   # request capabilities with illegal "channel"

configAntNetworkKeyJunkOK     = "46 02 B9 A5 21 FB 00 00 00 00 00 00"
configAntNetworkKeyJunkNotOK  = "46 03 00 01 02 03 04 05 06 07"
configAntNetworkKeyJunkOK2    = "46 e2 B9 A5 21 FB"
configAntNetworkKeyJunkNotOK2 = "46 f2 B9 A5 12 23 34 45 56 67 78 89"

INVALID_MESSAGE = "40 01 00 28"

# rmmod usb_serial_simple usbserial

def device2name(device):
    return 'DEVICE ID %04x:%04x on Bus %03d Adresse %03d' % (device.idVendor, device.idProduct, device.bus, device.address)

def checksum(data):         #calulate message checksum
  xor = 0
  for b in data:
      xor = xor ^ b
  return xor

def byte2hex(data):
    return ' '.join(format(x, '02x') for x in data)

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'



def testR(testname, sendMsg, recvMsg, isEquals=True, ignoreErr=False):
    isnot = ""
    if isEquals != True:
        isnot = bcolors.OKBLUE+"NOT"+bcolors.ENDC

    test = bcolors.OKBLUE+">>"+testname+"<<"+bcolors.ENDC

    try:
        if sendMsg != None:
            assert dev.write(endpointOUT, sendMsg, timeout=usbWriteTimeout) == len(sendMsg)
            test = test +" SENT: "+byte2hex(sendMsg)
        else:
            test = test +" SENT: NONE          "

        if recvMsg != None:
            test = test + "|| "+isnot+" EXPECTED: "+byte2hex(recvMsg)
            dataRead = dev.read(endpointIN, 64, timeout=usbReadTimeout)

            test = test + "|| RECEIVED: "+byte2hex(dataRead)

            if (dataRead != recvMsg) == isEquals and ignoreErr:
                result = bcolors.WARNING + 'WARNING'+bcolors.ENDC + " " + test
                errorlist.append(result)
            elif (dataRead != recvMsg) == isEquals:
                result = bcolors.FAIL + 'FAIL'+bcolors.ENDC + " " + test
                errorlist.append(result)
            else:
                result = bcolors.OKGREEN + 'OK'+bcolors.ENDC + " " + test
        else:
            result = bcolors.OKGREEN + 'OK'+bcolors.ENDC + " " + test
            test = test + "|| NOWAIT"
    except Exception as ex:
        result = bcolors.FAIL + 'EXCEPTION '+bcolors.ENDC + " " + test + str(ex)
        errorlist.append(result)

    print(result)

def test(testname, sendMsg, recvMsg, isEquals=True, ignoreErr=False):
    testR(testname, sendMsg, recvMsg, isEquals, ignoreErr)

def I(x):
    return x

def M(x):
    return marshal(x)

def H(x):
    return bytes.fromhex(x)

def reset():
    test("reset",              M(H(ctrlReset)), M(H("6f 20")))


def marshal(msgBody):
    msg = bytearray(msgBody)
    msg.insert(0, len(msg)-1)  # msg size (without sync, lengthfield, checksum, command byte)
    msg.insert(0, 0xa4)        # Sync byte
    msg.append(checksum(msg))
    return msg

def main():
    global dev, usbReadTimeout, usbWriteTimeout, endpointIN, endpointOUT, errorlist

    errorlist = [ ]

    dev = usb.core.find(idVendor=0x0fcf, idProduct=0x1008)

    for config in dev:
        print('Found # Interfaces', config.bNumInterfaces)
        for i in range(config.bNumInterfaces):
            if dev.is_kernel_driver_active(i):
                print("Force Kernel driver detach")
                dev.detach_kernel_driver(i)


    dev.reset()

    time.sleep(0.5)

    print(device2name(dev))
    print(dev)

    usbReadTimeout = 500
    usbWriteTimeout = 500

    endpointIN  = 0x81 # ANT USB endpointIN
    endpointOUT = 0x01 # ANT USB endpointOUT

    try: 
        dev.read(endpointIN, 64, timeout=usbReadTimeout)
        dev.read(endpointIN, 64, timeout=usbReadTimeout)
        dev.read(endpointIN, 64, timeout=usbReadTimeout)
        dev.read(endpointIN, 64, timeout=usbReadTimeout)
    except Exception:
        print("Nothing to drop in the beginning")

    # one initial reset without check - we can get 0x00 ot 0x20 as startup message
    test("reset1",              M(H(ctrlReset)), M(H("6f 00")), True, True)
    # next reset with check => restart with 0x20
    test("reset2",              M(H(ctrlReset)), M(H("6f 20")))

    # legal 0x4d commands - individual for one specific device
    # individual for 0x1008 (ANTUSB2) sticks
    test("requestAntVersion",   M(H(requestAntVersion)),  M(H("3e 41 50 32 55 53 42 31 2e 30 35 00")))



    test("requestCapabilities", M(H(requestCapabilities)),M(H("54 08 03 00 ba 36 00")))
    # only warning - we need a range here
    test("requestStackLimit",   M(H(requestStackLimit)),  M(H("55 b9 00")), True, True)
    test("requestSerial",       M(H(requestSerial)),      M(H("61 f4 2b 25 12")), True, True)
    test("requestFlashProtect", M(H(requestFlashProtect)),M(H("a7 01")))  # or a7 00
    test("requestGarminEsd",    M(H(requestGarminEsd)),   M(H("c6 a0 a1 a2 a3 a4")))

    # ######################################
    # Valid commands (without C7)
    # ######################################

    # 0x15 CHANNEL_IN_WRONG_STATE
    err15cmdAfterReset = [
        MESG_UNASSIGN_CHANNEL_ID,
        MESG_OPEN_CHANNEL_ID,
        MESG_CLOSE_CHANNEL_ID,
        MESG_BROADCAST_DATA_ID,
        MESG_ACKNOWLEDGED_DATA_ID,
        MESG_OPEN_RX_SCAN_ID,
        MESG_EXT_BROADCAST_DATA_ID,
        MESG_EXT_ACKNOWLEDGED_DATA_ID ]
    # 0x16 CHANNEL_NOT_OPENED
    err16cmdAfterReset = [ MESG_BURST_DATA_ID, MESG_EXT_BURST_DATA_ID ]
    # ignore RESET and USB-REFLASH
    ignore = [ MESG_SYSTEM_RESET_ID, MESG_SET_USB_INFO_ID ]
    # valid
    validCmd = [ 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x51, 0x53, MESG_ID_LIST_ADD_ID, 0x5a, 0x60, 0x63, 0x65, 0x66, 0x67, 0x6a, 0x6e, 0x70, 0x71, 0x75, 0xad ]


    # with legal channel but illegal network

    for cmd in range(0,0x100):
        # step over 0xc7 because this can reprogram your USB data
        if cmd in ignore or cmd in err15cmdAfterReset or cmd in err16cmdAfterReset or cmd in validCmd:
            continue
        answerErr28 = "40 00 {:02x} 28".format(cmd)
        req    = "{:02x} 00 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd1.1 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr28)))

    for cmd in validCmd:
        answerOk = "40 00 {:02x} 00".format(cmd)
        req    = "{:02x} 00 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd1.2 0x{:02x}".format(cmd),  M(H(req)), M(H(answerOk)))

    for cmd in err15cmdAfterReset:
        answerErr15 = "40 00 {:02x} 15".format(cmd)
        req    = "{:02x} 00 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd1.3 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr15)))

    for cmd in err16cmdAfterReset:
        answerErr16 = "40 00 {:02x} 16".format(cmd)
        req    = "{:02x} 00 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd1.4 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr16)))


    # with all 0x01

    for cmd in range(0,0x100):
        # step over 0xc7 because this can reprogram your USB data
        if cmd in ignore or cmd in err15cmdAfterReset or cmd in err16cmdAfterReset or cmd in validCmd:
            continue
        answerErr28 = "40 00 {:02x} 28".format(cmd)
        req    = "{:02x} 00 01 01 01 01 01 01 01 01".format(cmd)
        reset()
        test("Cmd2.1 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr28)))

    for cmd in validCmd:
        answerOk = "40 00 {:02x} 00".format(cmd)
        req    = "{:02x} 00 01 01 01 01 01 01 01 01".format(cmd)
        reset()
        test("Cmd2.2 0x{:02x}".format(cmd),  M(H(req)), M(H(answerOk)))

    for cmd in err15cmdAfterReset:
        answerErr15 = "40 00 {:02x} 15".format(cmd)
        req    = "{:02x} 00 01 01 01 01 01 01 01 01".format(cmd)
        reset()
        test("Cmd2.3 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr15)))

    for cmd in err16cmdAfterReset:
        answerErr16 = "40 00 {:02x} 16".format(cmd)
        req    = "{:02x} 00 01 01 01 01 01 01 01 01".format(cmd)
        reset()
        test("Cmd2.4 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr16)))

    # with all 0x08

    specials03 = { 0x42 : 0x29,  0x59 : 0x30, 0x5a: 0x30, 0x6e : 0x33}

    for cmd in range(0,0x100):
        # step over 0xc7 because this can reprogram your USB data
        if cmd in ignore or cmd in err15cmdAfterReset or cmd in err16cmdAfterReset or cmd in validCmd:
            continue
        answerErr28 = "40 00 {:02x} 28".format(cmd)
        req    = "{:02x} 00 08 08 08 08 08 08 08 08".format(cmd)
        reset()
        test("Cmd3.1 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr28)))

    for cmd in validCmd:
        if cmd in specials03:
            answerOk = "40 00 {:02x} {:02x}".format(cmd, specials03[cmd])
        else:
            answerOk = "40 00 {:02x} 00".format(cmd)
        req    = "{:02x} 00 08 08 08 08 08 08 08 08".format(cmd)
        reset()
        test("Cmd3.2 0x{:02x}".format(cmd),  M(H(req)), M(H(answerOk)))

    for cmd in err15cmdAfterReset:
        answerErr15 = "40 00 {:02x} 15".format(cmd)
        req    = "{:02x} 00 08 08 08 08 08 08 08 08".format(cmd)
        reset()
        test("Cmd3.3 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr15)))

    for cmd in err16cmdAfterReset:
        answerErr16 = "40 00 {:02x} 16".format(cmd)
        req    = "{:02x} 00 08 08 08 08 08 08 08 08".format(cmd)
        reset()
        test("Cmd3.4 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr16)))


    # with illegal channel
    for cmd in range(0,0x100):
        # step over 0xc7 because this can reprogram your USB data
        if cmd in ignore:
            continue
        answerErr28 = "40 08 {:02x} 28".format(cmd)
        req    = "{:02x} 08 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd4 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr28)))

    # with still valid channel
    specials04 = { 0x46 : 0x28 }

    for cmd in range(0,0x100):
        # step over 0xc7 because this can reprogram your USB data
        if cmd in ignore or cmd in err15cmdAfterReset or cmd in err16cmdAfterReset or cmd in validCmd:
            continue
        answerErr28 = "40 07 {:02x} 28".format(cmd)
        req    = "{:02x} 07 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd5.1 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr28)))

    for cmd in validCmd:
        if cmd in specials04:
            answerOk = "40 07 {:02x} {:02x}".format(cmd, specials04[cmd])
        else:
            answerOk = "40 07 {:02x} 00".format(cmd)
        req    = "{:02x} 07 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd5.2 0x{:02x}".format(cmd),  M(H(req)), M(H(answerOk)))

    for cmd in err15cmdAfterReset:
        answerErr15 = "40 07 {:02x} 15".format(cmd)
        req    = "{:02x} 07 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd5.3 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr15)))

    for cmd in err16cmdAfterReset:
        answerErr16 = "40 07 {:02x} 16".format(cmd)
        req    = "{:02x} 07 00 00 00 00 00 00 00 00".format(cmd)
        reset()
        test("Cmd5.4 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr16)))


    # with channel 0 and 0xff ff

    specialsFF = { 0x42 : 0x29,  0x59 : 0x30, 0x5a: 0x30, 0x6e : 0x33 }

    for cmd in range(0,0x100):
        # step over 0xc7 because this can reprogram your USB data
        if cmd in ignore or cmd in err15cmdAfterReset or cmd in err16cmdAfterReset or cmd in validCmd:
            continue
        answerErr28 = "40 00 {:02x} 28".format(cmd)
        req    = "{:02x} 00 ff ff ff ff ff ff ff ff".format(cmd)
        reset()
        test("Cmd6.1 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr28)))

    for cmd in validCmd:
        if cmd in specialsFF:
            answerOk = "40 00 {:02x} {:02x}".format(cmd, specialsFF[cmd])
        else:
            answerOk = "40 00 {:02x} 00".format(cmd)
        req    = "{:02x} 00 ff ff ff ff ff ff ff ff".format(cmd)
        reset()
        test("Cmd6.2 0x{:02x}".format(cmd),  M(H(req)), M(H(answerOk)))

    for cmd in err15cmdAfterReset:
        answerErr15 = "40 00 {:02x} 15".format(cmd)
        req    = "{:02x} 00 ff ff ff ff ff ff ff ff".format(cmd)
        reset()
        test("Cmd6.3 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr15)))

    for cmd in err16cmdAfterReset:
        answerErr16 = "40 00 {:02x} 16".format(cmd)
        req    = "{:02x} 00 ff ff ff ff ff ff ff ff".format(cmd)
        reset()
        test("Cmd6.4 0x{:02x}".format(cmd),  M(H(req)), M(H(answerErr16)))


    # ######################################
    # Request IDs
    # ######################################

    # test "illegal" MESG_REQUEST_ID (0x4d) commands
    for cmd in range(0,0x100):
        req    = "4d 00 {:02x}".format(cmd)
        reset()
        test("request 0x{:02x}".format(cmd),  M(H(req)), M(H("40 00 4d 28")), cmd not in valid_request_ids)

 # set ANT+ standard network key for NET #0
    # some general setting
    test("configTxPowerFF",     M(H(configTxPowerFF)),     M(H("40 00 47 00")))
    test("configAntNetworkKey", M(H(configAntNetworkKey)), M(H("40 00 46 00")))

    # some general data frame test (include emulation of hardware 'bugs')

    # ######################################
    # junk and fragment tests
    # ######################################

    zeros = "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
    junk  = "00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20"

    test("singleA4fragment-pretest", M(H(ctrlReset)), M(H("6f 20")))
    # OK -> same answer as last command
    testR("singleA4fragment", H("a4"), M(H("6f 20")))

    testR("junk1", H("00 11 22 33"),     M(H("ae 00 00 11 22")))
    testR("junk2", H("a5 ff ff ff"),     M(H("ae 00 a5 ff ff")))
    testR("longjunk", H(junk),         M(H("ae 00 "+junk[0:3*0x1f])))
    testR("longjunkA4", H("a4 "+junk),   M(H("ae 02 a4 "+junk[0:3*0x1e])))
    #
    test("longCheckOKMsgJunk", M(H(junk)), M(H(INVALID_MESSAGE)))
    test("longCheckOKMsgJunk", M(H(junk+" 21")), M(H(INVALID_MESSAGE)))
    test("longCheckOKMsgJunkTooLong", M(H(junk+" 21 22")), M(H("ae 03 a4 22 "+junk[0:3*0x1d])))
    # long valid message with zeros -> OK as long as it fits into one USB 64 bytes frame
    test("reset+Zeros",  M(H(ctrlReset))+H(zeros) , M(H("6f 20")))

    # Test junk channel and 'short' messages fragments

    # next reset with Junk (should work as long as "channel" (2. bytes) is < 8)
    test("resetJunkChannelOK",   M(H(ctrlResetJunkChannelOK)), M(H("6f 20")))
    # next reset with fragment (should still work)
    test("resetNoChannelOk",     M(H(ctrlResetNoChannel)), M(H("6f 20")))
    # next reset with Junk (should not work) => error for 'channel 8'
    test("resetJunkChannelErr",  M(H(ctrlResetJunkChannelErr)), M(H("40 08 4a 28")))
    # next reset with fragment (should not work, too) => error for 'channel 8'
    test("resetNoChannelNotOk",  M(H(ctrlResetNoChannel)), M(H("40 08 4a 28")))

    # test the 0x1f "masking" of the channel/network and the limit if channel (07) and network (02) - even if the command is no channel command
    test("requestCapabilitiesJunkOK",     M(H(requestCapabilitiesJunkOK)), M(H("54 08 03 00 ba 36 00")))
    test("requestCapabilitiesJunkOK1",    M(H(requestCapabilitiesJunkOK1)), M(H("54 08 03 00 ba 36 00")))
    test("requestCapabilitiesJunkNotOK1", M(H(requestCapabilitiesJunkNotOK1)), M(H("40 08 4d 28")))
    test("requestCapabilitiesJunkNotOK2", M(H(requestCapabilitiesJunkNotOK2)), M(H("40 17 4d 28")))
    test("requestCapabilitiesJunkNotOK3", M(H(requestCapabilitiesJunkNotOK3)), M(H("40 08 4d 28")))
    test("requestCapabilitiesJunkNotOK4", M(H(requestCapabilitiesJunkNotOK4)), M(H("40 1f 4d 28")))

    # test the 0x1f "masking" of the channel/network and the limit of network (02)
    test("configAntNetworkKeyJunkOK",     M(H(configAntNetworkKeyJunkOK)), M(H("40 02 46 00")))
    test("configAntNetworkKeyJunkNotOK",  M(H(configAntNetworkKeyJunkNotOK)), M(H("40 03 46 28")))
    test("configAntNetworkKeyJunkOK2",    M(H(configAntNetworkKeyJunkOK2)), M(H("40 02 46 00")))
    test("configAntNetworkKeyJunkNotOK2", M(H(configAntNetworkKeyJunkNotOK2)), M(H("40 12 46 28")))

    # ######################################
    # ....
    # ######################################

    # TODO tests with conflib 0x6e and 0x66 , also test that 0x66 changes 0x6e and vice versa

    # ######################################
    # ....
    # ######################################

    # TODO network tests (including max network number)

    # ######################################
    # ....
    # ######################################

    STATUS_CHANNEL_STATE_MASK   = (0x03)  # Channel state mask.
    STATUS_UNASSIGNED_CHANNEL   = (0x00)  # Indicates channel has not been assigned.
    STATUS_ASSIGNED_CHANNEL     = (0x01)  # Indicates channel has been assigned.
    STATUS_SEARCHING_CHANNEL    = (0x02)  # Indicates channel is active and in searching state.
    STATUS_TRACKING_CHANNEL     = (0x03)  # Indicates channel is active and in tracking state.

    channel = 0
    network = 0
    channelType = 0

    requestChannelStatus    = "4D {:02x} 52".format(channel)   # request channel status
    requestChannelId        = "4D {:02x} 51".format(channel)   # request channel status

    channelStatusUNASSIGNED = "52 {:02x} 00".format(channel)
    channelStatusASSIGNED   = "52 {:02x} 01".format(channel)
    channelStatusSEARCHING  = "52 {:02x} 02".format(channel)
    channelStatusTRACKING   = "52 {:02x} 00".format(channel)

    openChannel             = "4B {:02x}".format(channel)
    closeChannel            = "4C {:02x}".format(channel)
    unasignChannel          = "41 {:02x}".format(channel)

    openChannel_CH1             = "4B {:02x}".format(1)
    closeChannel_CH1            = "4C {:02x}".format(1)
    unasignChannel_CH1          = "41 {:02x}".format(1)
    openChannel_CH7             = "4B {:02x}".format(7)
    closeChannel_CH7            = "4C {:02x}".format(7)
    unasignChannel_CH7          = "41 {:02x}".format(7)

    CHANNEL_IN_WRONG_STATE_4open_CH1   = "40 {:02x} 4b 15".format(1) # error wrong state after OPEN CHANNEL

    CHANNEL_IN_WRONG_STATE_4open   = "40 {:02x} 4b 15".format(channel) # error wrong state after OPEN CHANNEL
    CHANNEL_IN_WRONG_STATE_4close  = "40 {:02x} 4c 15".format(channel) # error wrong state after CLOSE CHANNEL
    CHANNEL_IN_WRONG_STATE_4assign = "40 {:02x} 42 15".format(channel) # error wrong state after CLOSE CHANNEL
    CHANNEL_IN_WRONG_STATE_4unassign = "40 {:02x} 41 15".format(channel) # error wrong state after CLOSE CHANNEL
    CHANNEL_IN_WRONG_STATE_4broadcast = "40 {:02x} 4e 15".format(channel) # error wrong state after CLOSE CHANNEL
    INVALID_NETWORK_NUMBER    = "40 {:02x} 42 29".format(channel)     # wrong network after assign channel
    answerOk_4assign          = "40 {:02x} 42 00".format(channel)     # wrong network after assign channel
    answerOk_4unassign        = "40 {:02x} 41 00".format(channel)     # wrong network after assign channel
    answerOk_4open            = "40 {:02x} 4B 00".format(channel)     # wrong network after assign channel
    answerOk_4close           = "40 {:02x} 4C 00".format(channel)     # wrong network after assign channel
    answerOk_4channelId       = "40 {:02x} 51 00".format(channel)     # wrong network after assign channel

    answerOk_4assign_CH1          = "40 {:02x} 42 00".format(1)     # wrong network after assign channel
    answerOk_4unassign_CH1        = "40 {:02x} 41 00".format(1)     # wrong network after assign channel
    answerOk_4open_CH1            = "40 {:02x} 4B 00".format(1)     # wrong network after assign channel
    answerOk_4close_CH1           = "40 {:02x} 4C 00".format(1)     # wrong network after assign channel
    answerOk_4channelId_CH1       = "40 {:02x} 51 00".format(1)     # wrong network after assign channel

    answerOk_4assign_CH7          = "40 {:02x} 42 00".format(7)     # wrong network after assign channel
    answerOk_4unassign_CH7        = "40 {:02x} 41 00".format(7)     # wrong network after assign channel
    answerOk_4open_CH7            = "40 {:02x} 4B 00".format(7)     # wrong network after assign channel
    answerOk_4close_CH7           = "40 {:02x} 4C 00".format(7)     # wrong network after assign channel
    answerOk_4channelId_CH7       = "40 {:02x} 51 00".format(7)     # wrong network after assign channel

    CHANNEL_TYPE_SLAVE          = (0x00)  # 	Slave channel (PARAMETER_RX_NOT_TX).
    CHANNEL_TYPE_MASTER         = (0x10)  # 	Master channel (PARAMETER_TX_NOT_RX).
    CHANNEL_TYPE_SHARED_SLAVE   = (0x20)  # 	Shared slave channel (PARAMETER_RX_NOT_TX | PARAMETER_SHARED_CHANNEL).
    CHANNEL_TYPE_SHARED_MASTER  = (0x30)  # 	Shared master channel (PARAMETER_TX_NOT_RX | PARAMETER_SHARED_CHANNEL).
    CHANNEL_TYPE_SLAVE_RX_ONLY  = (0x40)  # 	Slave rx only channel (PARAMETER_RX_NOT_TX | PARAMETER_RX_ONLY).
    CHANNEL_TYPE_MASTER_TX_ONLY = (0x50)  # 	Master tx only channel (PARAMETER_TX_NOT_RX | PARAMETER_NO_TX_GUARD_BAND).

    assignChannelTypeFF    = "42 {:02x} {:02x} {:02x}".format(channel,0xff,0)       # OK channelType is masked with 0xf0
    assignChannelTypeEE    = "42 {:02x} {:02x} {:02x}".format(channel,0xee,0)       # OK channelType is masked with 0xf0
    assignChannelTypeNet2  = "42 {:02x} {:02x} {:02x}".format(channel,0x00,2)     # OK network 0-2
    assignChannelTypeNet3  = "42 {:02x} {:02x} {:02x}".format(channel,0x00,3)     # error netowrk
    assignChannelTypeNetF0 = "42 {:02x} {:02x} {:02x}".format(channel,0x00,0xF0) # error network

    assignChannelTypeNet0S  = "42 {:02x} {:02x} {:02x}".format(channel,CHANNEL_TYPE_SLAVE,0)     # OK network 0-2
    assignChannelTypeNet0M  = "42 {:02x} {:02x} {:02x}".format(channel,CHANNEL_TYPE_MASTER,0)     # OK network 0-2
    assignChannelTypeNet0S_CH1 = "42 {:02x} {:02x} {:02x}".format(1,CHANNEL_TYPE_SLAVE,0)     # OK network 0-2
    assignChannelTypeNet0M_CH1 = "42 {:02x} {:02x} {:02x}".format(1,CHANNEL_TYPE_MASTER,0)     # OK network 0-2
    assignChannelTypeNet2S_CH7 = "42 {:02x} {:02x} {:02x}".format(7,CHANNEL_TYPE_SLAVE,2)     # OK network 0-2
    assignChannelTypeNet2M_CH7 = "42 {:02x} {:02x} {:02x}".format(7,CHANNEL_TYPE_MASTER,2)     # OK network 0-2

    deviceNo = 0x7691
    deviceType = 0x0b
    transmissionType = 0x05
    period = 8182

    channelID_ffff_ff_ff = "51 {:02x} ff ff ff ff 00".format(channel)
    channelID_0000_00_00 = "51 {:02x} 00 00 00 00 00".format(channel)
    channelID_7691_0b_05 = "51 {:02x} {:02x} {:02x} {:02x} {:02x} 00".format(channel, (deviceNo>>0)&0xff,(deviceNo>>8)&0xff,deviceType,transmissionType)
    broadcast_datapage10 = "4e {:02x} 10 00 00 00 00 00 00 00".format(channel)
    # MESG_BROADCAST_DATA_ID         = (0x4E)
    # MESG_ACKNOWLEDGED_DATA_ID      = (0x4F)
    # MESG_BURST_DATA_ID             = (0x50)


    reset()

    # TODO network 0,1,2 config - wrong key, wrong channel ie. we do not want to receive anything in this part
    # MESG_CHANNEL_RADIO_FREQ_ID     = (0x45)
    # MESG_NETWORK_KEY_ID            = (0x46)
    # "43 00 00 20" channel period = 8192 (FE-C)


    test("reqChannelStatus1_1"  , M(H(requestChannelStatus)), M(H(channelStatusUNASSIGNED)))
    test("channelOpen1"         , M(H(openChannel)),          M(H(CHANNEL_IN_WRONG_STATE_4open)))
    test("reqChannelStatus1_2"  , M(H(requestChannelStatus)), M(H(channelStatusUNASSIGNED)))
    test("channelClose1"        , M(H(closeChannel)),         M(H(CHANNEL_IN_WRONG_STATE_4close)))
    test("reqChannelStatus1_3"  , M(H(requestChannelStatus)), M(H(channelStatusUNASSIGNED)))
    test("unassignChannel1"     , M(H(unasignChannel)),       M(H(CHANNEL_IN_WRONG_STATE_4unassign)))
    test("reqChannelStatus1_3"  , M(H(requestChannelStatus)), M(H(channelStatusUNASSIGNED)))
    test("unassignChannel1_2"   , M(H(unasignChannel)),       M(H(CHANNEL_IN_WRONG_STATE_4unassign)))

    test("assignChannel2_1"  , M(H(assignChannelTypeNet0M)), M(H(answerOk_4assign)))
    test("reqChannelStatus2_1" , M(H(requestChannelStatus)), M(H("52 00 11")))
    test("assignChannel2_2"  , M(H(assignChannelTypeNet0M)), M(H(CHANNEL_IN_WRONG_STATE_4assign)))
    test("reqChannelStatus2_2" , M(H(requestChannelStatus)), M(H("52 00 11")))
    test("channelClose2"   , M(H(closeChannel)),         M(H(CHANNEL_IN_WRONG_STATE_4close)))
    test("reqChannelStatus2_3" , M(H(requestChannelStatus)), M(H("52 00 11")))
    test("unassignChannel2"  , M(H(unasignChannel)), M(H(answerOk_4unassign)))
    test("reqChannelStatus2_4" , M(H(requestChannelStatus)), M(H(channelStatusUNASSIGNED)))

    test("assignChannel3_1"  , M(H(assignChannelTypeNet0M)), M(H(answerOk_4assign)))
    reset()
    test("reqChannelStatus3" , M(H(requestChannelStatus)), M(H(channelStatusUNASSIGNED)))
    test("assignChannel3_2"  , M(H(assignChannelTypeNet0M)), M(H(answerOk_4assign)))

    reset()
    test("assignChannelTypeFF"  , M(H(assignChannelTypeFF)), M(H(answerOk_4assign)))
    reset()
    test("assignChannelTypeEE"  , M(H(assignChannelTypeEE)), M(H(answerOk_4assign)))
    reset()
    test("assignChannelTypeNet2"  , M(H(assignChannelTypeNet2)), M(H(answerOk_4assign)))
    reset()
    test("assignChannelTypeNet3"  , M(H(assignChannelTypeNet3)), M(H(INVALID_NETWORK_NUMBER)))
    reset()
    test("assignChannelTypeNetF0"  , M(H(assignChannelTypeNetF0)), M(H(INVALID_NETWORK_NUMBER)))

    reset()
    test("channelID_ff"      , M(H(channelID_ffff_ff_ff)),          M(H(answerOk_4channelId)))
    test("reqChannelId5_1" , M(H(requestChannelId)), M(H("51 00 ff ff 7f ff")))
    test("channelID_00"      , M(H(channelID_0000_00_00)),          M(H(answerOk_4channelId)))
    test("reqChannelId5_1" , M(H(requestChannelId)), M(H("51 00 00 00 00 00")))

    test("channelID5"      , M(H(channelID_7691_0b_05)),          M(H(answerOk_4channelId)))
    test("reqChannelId5_1" , M(H(requestChannelId)), M(H("51 00 91 76 0b 05")))
    reset()
    test("reqChannelId5_1" , M(H(requestChannelId)), M(H("51 00 91 76 0b 05")))

    reset()
    test("broadcast_datapage10_1" , M(H(broadcast_datapage10)),     M(H(CHANNEL_IN_WRONG_STATE_4broadcast)))
    test("reqChannelStatus5_1" ,    M(H(requestChannelStatus)),     M(H(channelStatusUNASSIGNED)))
    test("assignChannel5"  ,        M(H(assignChannelTypeNet0M)),    M(H(answerOk_4assign)))
    test("reqChannelStatus5_2" ,    M(H(requestChannelStatus)),     M(H("52 00 11")))
    test("reqChannelId5_1" ,        M(H(requestChannelId)),         M(H("51 00 91 76 0b 05")))
    test("channelID5"      ,        M(H(channelID_7691_0b_05)),     M(H(answerOk_4channelId)))

    test("reqChannelStatus5_3" ,    M(H(requestChannelStatus)),     M(H("52 00 11")))
    test("reqChannelId5_2" ,        M(H(requestChannelId)),         M(H("51 00 91 76 0b 05")))
    test("broadcast_datapage10_2" , M(H(broadcast_datapage10)),         M(H("FF")))

    # to check zero consuming HACK
    if False:
      testR("zero" , H("00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"),         None)


    test("reqChannelStatus5_5" ,    M(H(requestChannelStatus)),     M(H("52 00 11")))
    test("reqChannelId5_3" ,        M(H(requestChannelId)),         M(H("51 00 91 76 0b 05")))
    test("channelOpen5"    ,        M(H(openChannel)),              M(H(answerOk_4open)))
    test("reqChannelStatus5_6" ,    M(H(requestChannelStatus)),     M(H("52 00 12")))
    test("broadcast_datapage10_3" , M(H(broadcast_datapage10)),          None)
    test("reqChannelId5_4" ,        M(H(requestChannelId)),         M(H("51 00 91 76 0b 05")))
    test("channelID5"      ,        M(H(channelID_7691_0b_05)),     M(H(answerOk_4channelId)))
    test("reqChannelId5_5" ,        M(H(requestChannelId)),         M(H("51 00 91 76 0b 05")))
    test("broadcast_datapage10_4" , M(H(broadcast_datapage10)),          None)

    reset()
    test("broadcast_datapage10_1" , M(H(broadcast_datapage10)),          M(H(CHANNEL_IN_WRONG_STATE_4broadcast)))
    test("assignChannel5"  , M(H(assignChannelTypeNet0M)), M(H(answerOk_4assign)))
    test("channelID_00"      , M(H(channelID_0000_00_00)),          M(H(answerOk_4channelId)))
    test("reqChannelId5_1" , M(H(requestChannelId)), M(H("51 00 00 00 00 00")))
    test("broadcast_datapage10_1" , M(H(broadcast_datapage10)),          None)
    test("reqChannelId5_1" , M(H(requestChannelId)), M(H("51 00 00 00 00 00")))



    # TODO channel test
    # TODO channels state machine tests
    # TODO including sending ACK/BROADCAST messages to assigned, opend and closed channel



    # TODO cont. scanning
    ctrlContScanning            = "5B 00 00"          # Enable Continous Scanning
    ctrlContScanningCH1         = "5B 01 00"          # Enable Continous Scanning

    # cont scan state machine 
    reset()
    test("ContScan0_0"  , M(H(assignChannelTypeNet0S)), M(H(answerOk_4assign)))
    test("ContScan0_1"  , M(H(ctrlContScanning)), M(H("40 00 5b 00")))   # OK
    test("ContScan0_2"  , M(H(requestChannelStatus)), M(H(channelStatusSEARCHING)))  # 
    test("ContScan0_3"  , M(H(openChannel)),            M(H(CHANNEL_IN_WRONG_STATE_4open)))
    test("ContScan0_4"        , M(H(closeChannel)),         M(H(answerOk_4close)))
    test("ContScan0_4b"  , None, M(H("40 00 01 07")))   # event CLOSED

    test("ContScan0_5"  , M(H(requestChannelStatus)), M(H(channelStatusASSIGNED)))
    test("ContScan0_6"     , M(H(unasignChannel)),       M(H(answerOk_4unassign)))

    reset()
    test("ContScan1_0"  , M(H(assignChannelTypeNet0S_CH1)), M(H(answerOk_4assign_CH1)))
    test("ContScan1_1"  , M(H(ctrlContScanningCH1)), M(H("40 01 5b 15")))   # Not OK for Channel 1 because assign was on channel 1

    reset()
    test("ContScan2_0"  , M(H(assignChannelTypeNet0S)), M(H(answerOk_4assign)))
    test("ContScan2_1"  , M(H(ctrlContScanningCH1)), M(H("40 01 5b 00")))   # OK for Channel 1 because assign was on channel 0 and we would expect data on channel 0
    # TODO trigger data to check receive
    test("ContScan2_2"  , M(H(ctrlContScanning)), M(H("40 00 5b 19")))   # CLOSE_ALL_CHANNELS event 
    test("ContScan2_3"  , M(H(ctrlContScanningCH1)), M(H("40 01 5b 19")))   # CLOSE_ALL_CHANNELS event 
    test("ContScan2_4"  , M(H(assignChannelTypeNet0S_CH1)), M(H(answerOk_4assign_CH1)))
    test("ContScan2_5"  , M(H(openChannel_CH1)),            M(H(CHANNEL_IN_WRONG_STATE_4open_CH1)))
    test("ContScan2_6"  , M(H(unasignChannel_CH1)), M(H(answerOk_4unassign_CH1)))
    test("ContScan2_7"  , M(H(assignChannelTypeNet0M_CH1)), M(H(answerOk_4assign_CH1)))
    test("ContScan2_8"  , M(H(openChannel_CH1)),            M(H(CHANNEL_IN_WRONG_STATE_4open_CH1)))
    test("ContScan2_9"  , M(H(unasignChannel_CH1)), M(H(answerOk_4unassign_CH1)))

    test("ContScan2_A"  , M(H(assignChannelTypeNet2M_CH7)), M(H(answerOk_4assign_CH7)))
    test("ContScan2_B"  , M(H(openChannel_CH7)),            M(H(CHANNEL_IN_WRONG_STATE_4open_CH1)))

    # TODO timeout scanning

    # TODO network key tests
    # TODO ANT FE-C tests (emulation)

    print("======================================")
    print("=== Summary                        ===")
    print("======================================")
    for i in errorlist:
        print(i)


if __name__ == '__main__':
    main()
