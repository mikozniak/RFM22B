#-*- coding: utf8 -*-
u'''
    rfm22
    ~~~~~

    Driver for RFM22 module connected to Raspberry PI using wiringpi2 and RPIO.
    @version: 0.1
    @license: GNU GPL 
    @author: Maximilian Betz
   
'''
#Raspberry Pi specific imports:
u'''Wiringpi2 for SPI and basic GPIO.'''
#import wiringpi2 as wiringpi
import wiringpi
u'''RPIO for wait for interrupt function.'''
import RPIO
#import RPi.GPIO as GPIO

# Python imports
import threading
import queue
import random
import time

u'''Pin definitions, '''
RFM22_SDN_GPIO_PIN =  5 #24 BCM GPIO

u'''Nirq pin number defintion. wiringpi uses the 
RFM22_NIRQ_GPIO_PIN, RPIO uses RFM""_NIRQ_GPIO_BCM_PIN.'''
RFM22_NIRQ_GPIO_PIN = 6 #25 BCM GPIO
RFM22_NIRQ_GPIO_BCM_PIN = 25

RFM22_TXEN_GPIO_PIN    = 1 #18 BCM GPIO 
RFM22_RXEN_GPIO_PIN = 4 #23 BCM GPIO
RFM22_SPI_CHANNEL = 0 

u'''Definitions for wiringpi module.'''
GPIO_INT_EDGE_SETUP = 0
GPIO_INT_EDGE_FALLING = 1
GPIO_INT_EDGE_RISING = 2
GPIO_INT_EDGE_BOTH = 3
INPUT = 0
OUTPUT = 1
LOW = 0
HIGH = 1
PUD_OFF = 0
PUD_DOWN = 1
PUD_UP = 2

class Rfm22Error(Exception):
    u'''Rfm22Error exception class to handle rfm22 specific exceptions.'''
    def __init__(self, value):
        self.value = value
        def __str__(self):
            return repr(self.value)

def debug_log(x, b= None, c = None):
    u'''Debug print function wrapper.'''
    #return
    if b == None:
        print (x)
    elif c == None:
        print (x, b)
    else:
        print (x,b,c)
        
def nirq_callback(gpio_id, val):
    u'''Callback function for RPIO module. Function gets
    called when nirq interrupt occurs.'''
    debug_log("nirq callback gpio %s: %s" % (gpio_id, val),time.strftime('%X'))
    RPIO.stop_waiting_for_interrupts()

class Rfm22(threading.Thread):
    u'''The Rfm22 class provides functions to send and receive data via the
    connected RFM22 module. Reception and Transmission of data is done via
    queues. State machine and event processing and communication with the RFM22 
    module is completely handled in an extra thread.'''
    
    u'''Register address definitions.'''
    __DEVICE_TYPE                           = 0x00
    __DEVICE_VERSION                        = 0x01
    __DEVICE_STATUS                         = 0x02
    __INTERRUPT_STATUS_1                    = 0x03
    __INTERRUPT_STATUS_2                    = 0x04
    __INTERRUPT_ENABLE_1                    = 0x05
    __INTERRUPT_ENABLE_2                    = 0x06
    __OPERATING_FUNCTION_CONTROL_1          = 0x07
    __OPERATING_FUNCTION_CONTROL_2          = 0x08
    __CRYSTAL_OSCILLATOR_LOAD               = 0x09
    __MICROCONTROLLER_CLOCK_OUTPUT          = 0x0A
    __GPIO0_CONFIGURATION                   = 0x0B
    __GPIO1_CONFIGURATION                   = 0x0C
    __GPIO2_CONFIGURATION                   = 0x0D
    __I_O_PORT_CONFIGURATION                = 0x0E
    __ADC_CONFIGURATION                     = 0x0F
    __ADC_SENSOR_AMPLIFIER_OFFSET           = 0x10
    __ADC_VALUE                             = 0x11
    __TEMPERATURE_SENSOR_CONTROL            = 0x12
    __TEMPERATURE_VALUE_OFFSET              = 0x13
    __WAKE_UP_TIMER_PERIOD_1                = 0x14
    __WAKE_UP_TIMER_PERIOD_2                = 0x15
    __WAKE_UP_TIMER_PERIOD_3                = 0x16
    __WAKE_UP_TIMER_VALUE_1                 = 0x17
    __WAKE_UP_TIMER_VALUE_2                 = 0x18
    __LOW_DUTY_CYCLE_MODE_DURATION          = 0x19
    __LOW_BATTERY_DETECTION_THRESHOLD       = 0x1A
    __BATTERY_VOLTAGE_LEVEL                 = 0x1B
    __IF_FILTER_BANDWIDTH                   = 0x1C
    __AFC_LOOP_GEARSHIFT_OVERRIDE           = 0x1D
    __AFC_TIMING_CONTROL                    = 0x1E
    __CLOCK_RECOVERY_GEARSHIFT_OVERRIDE     = 0x1F
    __CLOCK_RECOVERY_OVERSAMPLING_RATIO     = 0x20
    __CLOCK_RECOVERY_OFFSET_2               = 0x21
    __CLOCK_RECOVERY_OFFSET_1               = 0x22
    __CLOCK_RECOVERY_OFFSET_0               = 0x23
    __CLOCK_RECOVERY_TIMING_LOOP_GAIN_1     = 0x24
    __CLOCK_RECOVERY_TIMING_LOOP_GAIN_0     = 0x25
    __RECEIVED_SIGNAL_STRENGTH_INDICATOR    = 0x26
    __RSSI_THRESSHOLF_FOR_CLEAR_CHANNEL_INDICATOR = 0x27
    __ANTENNA_DIVERSITY_REGISTER_1          = 0x28
    __ANTENNA_DIVERSITY_REGISTER_2          = 0x29
    __AFC_LIMITER                           = 0x2A
    __AFC_CORRECTION_READ                   = 0x2B
    __OOK_COUNTER_VALUE_1                   = 0x2C
    __OOK_COUNTER_VALUE_2                   = 0x2D
    __SLICER_PEAK_HOLD                      = 0x2E
    # Register 0x2F reserved
    __DATA_ACCESS_CONTROL                   = 0x30
    __EzMAC_STATUS                          = 0x31
    __HEADER_CONTROL_1                      = 0x32
    __HEADER_CONTROL_2                      = 0x33
    __PREAMBLE_LENGTH                       = 0x34
    __PREAMBLE_DETECTION_CONTROL            = 0x35
    __SYNC_WORD_3                           = 0x36
    __SYNC_WORD_2                           = 0x37
    __SYNC_WORD_1                           = 0x38
    __SYNC_WORD_0                           = 0x39
    __TRANSMIT_HEADER_3                     = 0x3A
    __TRANSMIT_HEADER_2                     = 0x3B
    __TRANSMIT_HEADER_1                     = 0x3C
    __TRANSMIT_HEADER_0                     = 0x3D
    __TRANSMIT_PACKET_LENGTH                = 0x3E
    __CHECK_HEADER_3                        = 0x3F
    __CHECK_HEADER_2                        = 0x40
    __CHECK_HEADER_1                        = 0x41
    __CHECK_HEADER_0                        = 0x42
    __HEADER_ENABLE_3                       = 0x43
    __HEADER_ENABLE_2                       = 0x44
    __HEADER_ENABLE_1                       = 0x45
    __HEADER_ENABLE_0                       = 0x46
    __RECEIVED_HEADER_3                     = 0x47
    __RECEIVED_HEADER_2                     = 0x48
    __RECEIVED_HEADER_1                     = 0x49
    __RECEIVED_HEADER_0                     = 0x4A
    __RECEIVED_PACKET_LENGTH                = 0x4B
    # Registers 0x4C-4E reserved
    __ADC8_CONTROL                          = 0x4F
    # Registers 0x50-5F reserved
    __CHANNEL_FILTER_COEFFICIENT_ADDRESS    = 0x60
    # Register 0x61 reserved
    __CRYSTAL_OSCILLATOR_CONTROL_TEST       = 0x62
    # Registers 0x63-68 reserved
    __AGC_OVERRIDE_1                        = 0x69
    # Registers 0x6A-0x6C reserved
    __TX_POWER                              = 0x6D
    __TX_DATA_RATE_1                        = 0x6E
    __TX_DATA_RATE_0                        = 0x6F
    __MODULATION_MODE_CONTROL_1             = 0x70
    __MODULATION_MODE_CONTROL_2             = 0x71
    __FREQUENCY_DEVIATION                   = 0x72
    __FREQUENCY_OFFSET_1                    = 0x73
    __FREQUENCY_OFFSET_2                    = 0x74
    __FREQUENCY_BAND_SELECT                 = 0x75
    __NOMINAL_CARRIER_FREQUENCY_1           = 0x76
    __NOMINAL_CARRIER_FREQUENCY_0           = 0x77
    # Register 0x78 reserved
    __FREQUENCY_HOPPING_CHANNEL_SELECT      = 0x79
    __FREQUENCY_HOPPING_STEP_SIZE           = 0x7A
    # Register 0x7B reserved
    __TX_FIFO_CONTROL_1                     = 0x7C
    __TX_FIFO_CONTROL_2                     = 0x7D
    __RX_FIFO_CONTROL                       = 0x7E
    __FIFO_ACCESS                           = 0x7F
        
    u'''Packet sync bytes. RFM22 on the other end must 
    have the same configuration.'''
    __PACKET_SYNC_BYTE_3 = 0x2D
    __PACKET_SYNC_BYTE_2 = 0xD4
    __PACKET_SYNC_BYTE_1 = 0xAA
    __PACKET_SYNC_BYTE_0 = 0xAA
        
    __TRANSMIT_HEADER_BYTE_3 = 0xFC
    __TRANSMIT_HEADER_BYTE_2 = 0x4E
    __TRANSMIT_HEADER_BYTE_1 = 0xA5
    __TRANSMIT_HEADER_BYTE_0 = 0xC8
    
    u'''Interrupt Status 1 Flag definitions'''
    __INTERRUPT_STATUS_1_FLAG_PACKET_TRANSMITTED = 0x04
    __INTERRUPT_STATUS_1_FLAG_PACKET_RECEIVED = 0x02
    __INTERRUPT_STATUS_1_FLAG_FIFO_UNDER_OVER_FLOW = 0x80
    __INTERRUPT_STATUS_1_FLAG_CRC_ERROR = 0x01
    __INTERRUPT_STATUS_1_FLAG_EXTERNAL_ISR = 0x08
    __INTERRUPT_STATUS_1_FLAG_RX_FIFO_ALMOST_FULL = 0x10
    __INTERRUPT_STATUS_1_FLAG_TX_FIFO_ALMOST_EMPTY = 0x20
    __INTERRUPT_STATUS_1_FLAG_TX_FIFO_ALMOST_FULL = 0x40
    
    u'''Interrupt Status 2 Flag definitions'''
    __INTERRUPT_STATUS_2_FLAG_RSSI = 0x10
    
    u'''Interrupt Flag initiated by send request and not from RFM22.'''
    __INTERRUPT_STATUS_TX_REQUEST = 0x100
    
    u'''Maximum packet length  FIFO 64 - 2 Byte for CRC.'''
    __MAXIMUM_PACKET_LENGTH = 62  
    __RSSI_CLEAR_CHANNEL_THRESHOLD = 100
        
    u'''State definitions for state machine handling.'''  
    __STATE_SENDING = 1
    __STATE_RECEIVING = 2 
        
    def __init__(self):
        self.__state =  self.__STATE_RECEIVING 
        self.__rssi = 0
        self.__thread_nirq_event = threading.Event()
        self.__tx_queue = queue.Queue()
        self.__rx_queue = queue.Queue()
        u'''Initialise parent class, necessary for 
        threading module.'''
        super(Rfm22, self).__init__()
        
    def __send_data(self, tx_bytes):
        u'''Converts the tx_byte list to a string and
        sends the string via the SPI interface.'''
        data =  ""
        rx_data = []
        for x in tx_bytes:
            data = data + chr(x)
        wiringpi.wiringPiSPIDataRW(0,data); 
        for x in data:
            rx_data.append(ord(x))
        return rx_data
    
    def __command(self, address, data):
        u'''Sends the address and the data via the 
        SPI interface.'''
        rx_data = None
        if type(address) is int:
            if type(data) is int:
                tx_bytes = address, data
                rx_data = self.__send_data(tx_bytes)
            else:
                debug_log("__command Data is no int")
                
        else:
            debug_log("__command Address is no int")
        return rx_data
        
    def __write(self, address, data):
        u'''Writes one byte of data to the RFM22 register 
        at the given address.'''
        ret = self.__command((0x80 | address), data)
        return ret
    
    def __read(self, address):
        u'''Reads one byte of data from the specified address.'''
        return self.__command((address & 0x7f), 0xff)
    
    def __sdn_on(self):
        u'''Sets the sdn pin to 0, to enable the RFM22
        module.'''
        wiringpi.digitalWrite(RFM22_SDN_GPIO_PIN,0)
        
    def __sdn_off(self):
        u'''Sets the sdn pin to 1, to disable the RFM22
        module. Used to reset the module before 
        initialisation.'''
        wiringpi.digitalWrite(RFM22_SDN_GPIO_PIN,1)
        
    def __rx_ant_on(self):
        u'''RX_ANT pin enable. May not necessary, 
        depending on RFM22 version.'''
        wiringpi.digitalWrite(RFM22_RXEN_GPIO_PIN,1)
    
    def __rx_ant_off(self):
        u'''RX_ANT pin disable. May not necessary, 
        depending on RFM22 version.'''
        wiringpi.digitalWrite(RFM22_RXEN_GPIO_PIN,0)
        
    def __tx_ant_on(self):
        u'''TX_ANT pin enable. May not necessary, 
        depending on RFM22 version.'''
        wiringpi.digitalWrite(RFM22_TXEN_GPIO_PIN,1)
        
    def __tx_ant_off(self):
        u'''TX_ANT pin disable. May not necessary, 
        depending on RFM22 version.'''
        wiringpi.digitalWrite(RFM22_TXEN_GPIO_PIN,0)
        
    def __rx_enable(self):
        u'''Disable tx antenna circuit. Enable rx antenna circuit
        May not necessary with latest RFM22 version.'''
        self.__tx_ant_off() 
        self.__rx_ant_on()
        
        u'''Clear interrupt status flags'''
        self.__read(self.__INTERRUPT_STATUS_1)
        self.__read(self.__INTERRUPT_STATUS_2)
        
        u'''Switch module state machine to ready mode'''
        self.__write(self.__OPERATING_FUNCTION_CONTROL_1, 0x01)
        
        u'''Clear RX FIFO'''
        self.__write(self.__OPERATING_FUNCTION_CONTROL_2, 0x03)
        self.__write(self.__OPERATING_FUNCTION_CONTROL_2, 0x00)
        
        u'''Clear TX FIFO'''
        self.__write(self.__OPERATING_FUNCTION_CONTROL_2, 0x02)
        self.__write(self.__OPERATING_FUNCTION_CONTROL_2, 0x00)
        
        u'''Clear interrupt status'''
        self.__read(self.__INTERRUPT_STATUS_1);
        self.__read(self.__INTERRUPT_STATUS_2);
        
        u'''To ready mode RX on!'''
        self.__write(self.__OPERATING_FUNCTION_CONTROL_1, 0x05)

    def __tx_enable(self):
        u'''Enable tx antenna circuit. Disable rx antenna circuit
        May not necessary with latest RFM22 version.'''
        self.__rx_ant_off() 
        self.__tx_ant_on()   
        
        u'''To ready mode, TX on!'''
        self.__write(self.__OPERATING_FUNCTION_CONTROL_1, 0x09)
        
    def get_rx_data(self, block = True, timeout = None):
        u'''Function to get data which has been received by
        the RFM22 module. 
        @return: List with bytes of one received package. Returns 
        an empty list if nothing has been received  '''
        try:
            return self.__rx_queue.get(block, timeout)
        except:
            return []
            
    def trigger_tx_event(self):
        u'''Interrupts the waiting for an NIRQ (External interrupt
        from RFM22) event. Necessary to send Packet immediately after
        posting to the queue.'''
        RPIO.stop_waiting_for_interrupts()
        
    def put_tx_data(self, data, block = True, timeout = None ):
        u'''Checks if the data is in a valid format. Sends the 
        valid data via the tx queue to the RFM22 handling thread
        and 
        @param data: list with bytes. Maximum length = __MAXIMUM_PACKET_LENGTH
        @param block: @see queue.put function.
        @param timeout: @see queue.put function. '''
        for x in data:
            if (x < 0 or x > 255):
                raise Rfm22Error('List can only contain bytes with value between 0 and 255!')
        
        self.__tx_queue.put(data, block, timeout)
        u'''Stop waiting for nirq event.'''
        self.trigger_tx_event()
                          
    def initialise(self):
        u'''Initialises the gpio pins, the SPI interface and configures
        the rfm22 module
        '''
        wiringpi.wiringPiSetup() 
        wiringpi.wiringPiSPISetup(RFM22_SPI_CHANNEL,10000000) 
        RPIO.setup(RFM22_NIRQ_GPIO_BCM_PIN, RPIO.IN)
        RPIO.cleanup_interrupts()
        RPIO.add_interrupt_callback(RFM22_NIRQ_GPIO_BCM_PIN, nirq_callback, edge='falling')
         
        wiringpi.pinMode(RFM22_SDN_GPIO_PIN, OUTPUT)
        wiringpi.pinMode(RFM22_TXEN_GPIO_PIN, OUTPUT)
        wiringpi.pinMode(RFM22_RXEN_GPIO_PIN, OUTPUT)
          
        self.__rx_ant_off()
        self.__tx_ant_off()
        self.__sdn_off()
        time.sleep(0.1) #reset module
        self.__sdn_on()
        #Wait 100ms to give rfm22 time to wake up
        time.sleep(0.1)
       
        #Read out chip version via SPI
        version = self.__read(self.__DEVICE_VERSION)
        debug_log("RFM22 chip version:", version)
        if (version[1] != 0):
            debug_log("SPI Communication working!")

        #Reset all registers, software reset
        self.__write(self.__OPERATING_FUNCTION_CONTROL_1, 0x80)
        
        #Wait 20ms to let the module boot up
        time.sleep(0.02)

        u'''Enable FIFO under/overflow, external, packet send,
        received and crc error interrupt.'''
        self.__write(self.__INTERRUPT_ENABLE_1, 0x87)
        
        u'''Enable rssi interrupt'''
        #self.__write(self.__INTERRUPT_ENABLE_2, 0x10) #Raspberry PI does not react fast enough to read correct value
        
        u'''Switch to ready mode.  200us to tx or rx'''
        self.__write(self.__OPERATING_FUNCTION_CONTROL_1, 0x01)

        self.__write(self.__CRYSTAL_OSCILLATOR_LOAD, 0x7f);  
    
        u'''Enable AFC'''
        self.__write(self.__AFC_LOOP_GEARSHIFT_OVERRIDE, 0x40);
    
        u'''Set AFC timing'''
        self.__write(self.__AFC_TIMING_CONTROL, 0x0A);  
        self.__write(self.__CLOCK_RECOVERY_GEARSHIFT_OVERRIDE, 0x05);
    
        u'''Set IF filter bandwidth'''
        self.__write(self.__IF_FILTER_BANDWIDTH, 0x9A);
    
        u'''Set clock recovery oversampling rate'''
        self.__write(self.__CLOCK_RECOVERY_OVERSAMPLING_RATIO, 0x3C);
        self.__write(self.__CLOCK_RECOVERY_OFFSET_2, 0x02);
        self.__write(self.__CLOCK_RECOVERY_OFFSET_1, 0x22);
        self.__write(self.__CLOCK_RECOVERY_OFFSET_0, 0x22);
    
        self.__write(self.__CLOCK_RECOVERY_TIMING_LOOP_GAIN_1, 0x07);
        self.__write(self.__CLOCK_RECOVERY_TIMING_LOOP_GAIN_0, 0xFF);
    
        u'''Set AFC Limiter'''
        self.__write(self.__AFC_LIMITER, 0x48);
    
        u'''Set RSSI threshold for clear channel assessment'''
        self.__write(self.__RSSI_THRESSHOLF_FOR_CLEAR_CHANNEL_INDICATOR, self.__RSSI_CLEAR_CHANNEL_THRESHOLD);
    
        u'''RX, TX packet handling, enable CCIT CRC'''
        self.__write(self.__DATA_ACCESS_CONTROL, 0xAC);
    
        u'''Header check, valid for broadcast or check byte'''
        self.__write(self.__HEADER_CONTROL_1, 0xFF);
    
        u'''4 byte header (Header 3,2,1,0) and 2 byte sync (Synchronization Word 3,2) '''
        self.__write(self.__HEADER_CONTROL_2, 0x42);
    
        u'''Synchronization words'''
        self.__write(self.__SYNC_WORD_3, self.__PACKET_SYNC_BYTE_3);
        self.__write(self.__SYNC_WORD_2, self.__PACKET_SYNC_BYTE_2);
        self.__write(self.__SYNC_WORD_1, self.__PACKET_SYNC_BYTE_1);
        self.__write(self.__SYNC_WORD_0, self.__PACKET_SYNC_BYTE_0);
    
        u'''Transmit header'''
        self.__write(self.__TRANSMIT_HEADER_3, self.__TRANSMIT_HEADER_BYTE_3);
        self.__write(self.__TRANSMIT_HEADER_2, self.__TRANSMIT_HEADER_BYTE_2);
        self.__write(self.__TRANSMIT_HEADER_1, self.__TRANSMIT_HEADER_BYTE_1);
        self.__write(self.__TRANSMIT_HEADER_0, self.__TRANSMIT_HEADER_BYTE_0);
    
        self.__write(self.__CHECK_HEADER_3, self.__TRANSMIT_HEADER_BYTE_3);
        self.__write(self.__CHECK_HEADER_2, self.__TRANSMIT_HEADER_BYTE_2);
        self.__write(self.__CHECK_HEADER_1, self.__TRANSMIT_HEADER_BYTE_1);
        self.__write(self.__CHECK_HEADER_0, self.__TRANSMIT_HEADER_BYTE_0);
    
        u'''Check header enable mask, 0xFF means all bits of the corresponding
         * header will be checked. '''
        self.__write(self.__HEADER_ENABLE_3, 0xFF);
        self.__write(self.__HEADER_ENABLE_2, 0xFF);
        self.__write(self.__HEADER_ENABLE_1, 0xFF);
        self.__write(self.__HEADER_ENABLE_0, 0xFF);
    
        u'''Switch on automatic RX gain control'''
        self.__write(self.__AGC_OVERRIDE_1, 0x60);
        u'''TODO: example check agc override Register 0x6a does not exist. Data sheet wrong?'''
    
        u'''Set TX power and  LNA controller on if RFM22 != TX
         * TXPower         equals
         * -1dbm        0x00
         *  2dbm        0x01   set to 2dbm
         *  5dbm        0x02
         *  8dbm        0x03
         *  11dbm       0x04
         *  14dbm       0x05
         *  17dbm       0x06
         *  20dbm       0x07
         * '''
        self.__write(self.__TX_POWER, (0x01|0x80));
    
        u'''Set TX data rate to 0x199a =  100 kbps.
         * Datarate calculation:
         * TX_DR = 10^6 * 0x199a / 2^16 '''
        self.__write(self.__TX_DATA_RATE_1, 0x19); #Bits 15:8
        self.__write(self.__TX_DATA_RATE_0, 0x9a); #Bits 7:0
    
        u'''Modulation control, manchester data inversion.'''    
        self.__write(self.__MODULATION_MODE_CONTROL_1, 0x0C);
    
        u'''Modulation control, FIFO Mode, GFSK '''
        self.__write(self.__MODULATION_MODE_CONTROL_2, 0x23);
    
        u'''Frequency deviation 50kHz
         * deviation = 625 * 0x50'''
        self.__write(self.__FREQUENCY_DEVIATION, 0x50);
    
        u'''Select frequency band, high band = >480 < 960 MHz,
         * 860- 879.9MHz
         * Regulations: 869,4 869,65 at 500mW ERP < 10%
         * '''
        self.__write(self.__FREQUENCY_BAND_SELECT, 0x73);
    
        self.__write(self.__NOMINAL_CARRIER_FREQUENCY_1, 0x76);
    
        self.__write(self.__NOMINAL_CARRIER_FREQUENCY_0, 0xC0);
        u'''Set carrier '''
        debug_log(u"RFM22 initialised")
                          
        
    def __identify_nirq_event(self):
        return self.__read(self.__INTERRUPT_STATUS_1)[1]
               
    def run(self):
        u'''Handles the state machine reacts on receive and
        transmit events.'''
        u'''Enable receiver.'''
        self.__rx_enable()
        self.__state = self.__STATE_RECEIVING
        
        debug_log("Start..", time.strftime('%X'))
        while(True):
            
            u'''Only wait for Interrupt if no TX 
            requests pending.'''
            if(self.__tx_queue.empty() == True):
                RPIO.wait_for_interrupts() #Wait for NIRQ event
            
            u'''Alternative busy waiting. Takes a lot of CPU time.'''
            #while(RPIO.input(RFM22_NIRQ_GPIO_BCM_PIN) == True):
            #    if(self.__tx_queue.empty() == False):
            #        break
                    
            debug_log("Exit wait for interrupt")  
            u'''Get Interrupt 1 Flags'''
            if (RPIO.input(RFM22_NIRQ_GPIO_BCM_PIN)) == True:
                u'''True means no Interrupt from RFM22 initiated.
                Tx requests are pending.'''
                event = self.__INTERRUPT_STATUS_TX_REQUEST
            else:
                u'''Interrupt by RFM22 read interrupt status.'''
                event = self.__identify_nirq_event()
                
            u'''Handle Event:'''             
            if self.__state == self.__STATE_SENDING:
                debug_log("State Sending!")
                if(event & self.__INTERRUPT_STATUS_1_FLAG_PACKET_TRANSMITTED):
                    u'''Clear event and enable receiver.'''
                    event = event & ~self.__INTERRUPT_STATUS_1_FLAG_FIFO_UNDER_OVER_FLOW
                    self.__rx_enable()
                    self.__state = self.__STATE_RECEIVING
                    debug_log("Packet Transmitted", time.strftime('%X'))
                
                elif(event & self.__INTERRUPT_STATUS_1_FLAG_FIFO_UNDER_OVER_FLOW):
                    u'''Clear event reset FIFOS and enable receiver'''
                    event = event & ~self.__INTERRUPT_STATUS_1_FLAG_FIFO_UNDER_OVER_FLOW
                    register_data = self.__read(self.__OPERATING_FUNCTION_CONTROL_2)[1]
                    self.__write(self.__OPERATING_FUNCTION_CONTROL_2, register_data | 0x01)
                    self.__write(self.__OPERATING_FUNCTION_CONTROL_2, register_data & 0xFE)
                    self.__rx_enable()
                    self.__state = self.__STATE_RECEIVING
                    debug_log("Fifo error while sending. Receiving", time.strftime('%X'))
                else:
                    u'''Clear event'''
                    event = 0
                         
            elif self.__state & self.__STATE_RECEIVING:
                debug_log("State Receiving!")
                if (event & self.__INTERRUPT_STATUS_1_FLAG_PACKET_RECEIVED):
                    u'''clear event, get packet, clear fifo, enable rx mode 
                    again.'''         
                    event = event & ~self.__INTERRUPT_STATUS_1_FLAG_PACKET_RECEIVED
                    packet_length = self.__read(self.__RECEIVED_PACKET_LENGTH)[1]
                    data = [0xFF] * (packet_length)
                    data.insert(0, 0x7F & self.__FIFO_ACCESS) 
                    data = self.__send_data(data)[1:]
                    
                    u'''Received data is now stored in data. Do something 
                    with it!'''
                    debug_log("Received Data", data)
                    self.__rx_queue.put(data, block = False)
                    
                    debug_log("Packet received, Bytes:", packet_length, time.strftime('%X'))
                    self.__rx_enable()
                
                elif(event & self.__INTERRUPT_STATUS_1_FLAG_FIFO_UNDER_OVER_FLOW ):
                    u'''Clear event, reset fifos, enable receiving.'''
                    event = event & ~self.__INTERRUPT_STATUS_1_FLAG_FIFO_UNDER_OVER_FLOW
                    register_data = self.__read(self.__OPERATING_FUNCTION_CONTROL_2)[1]
                    self.__write(self.__OPERATING_FUNCTION_CONTROL_2, register_data | 0x02)
                    self.__write(self.__OPERATING_FUNCTION_CONTROL_2, register_data & 0xFD)
                    self.__rx_enable()
                    debug_log("FIFO error while receiving.")
                
                elif(event & self.__INTERRUPT_STATUS_TX_REQUEST):  
                    u'''Clear event.'''
                    event = event & ~self.__INTERRUPT_STATUS_TX_REQUEST
                    register_data = self.__read(self.__INTERRUPT_STATUS_2)[1]
                    if(register_data & self.__INTERRUPT_STATUS_2_FLAG_RSSI):
                        u'''Channel is not free, safe rssi'''
                        debug_log("Interrupt_status_2_reg", register_data)
                        self.__rssi = self.__read(self.__RECEIVED_SIGNAL_STRENGTH_INDICATOR)
                        debug_log("Channel not free, RSSI Level: ", self.__rssi)
                        
                        #TODO: add CSMA/CA algorithm with random delay time.
                        maximum_packet_time = 0.01
                        packet_delay = random.randint(1, 10)
                        delay_time = maximum_packet_time * packet_delay
                        #retrigger_timer = threading.Timer(delay_time, self.trigger_tx_event())
                        #retrigger_timer.start()
                        debug_log("Tx Retrigger mechanism delay !implement me! : ", delay_time)
                    else:
                        try:
                            u'''Get data from queue and configure packet handler packet 
                            length.'''
                            tx_data = self.__tx_queue.get(block = False)
                            length = len(tx_data)
                            
                            u'''Clear interrupt status'''
                            self.__read(self.__INTERRUPT_STATUS_1)
                            self.__read(self.__INTERRUPT_STATUS_2)
                            u'''Clear TX Fifo'''
                            self.__write(self.__OPERATING_FUNCTION_CONTROL_2, register_data | 0x02)
                            self.__write(self.__OPERATING_FUNCTION_CONTROL_2, register_data & 0x00)

                            if(length <= self.__MAXIMUM_PACKET_LENGTH):
                                u'''If packet is not to large. Write data to Fifo, set length and enable
                                TX packet transmission.'''
                                self.__write(self.__TRANSMIT_PACKET_LENGTH, length)
                                tx_data.insert(0, 0x80 |self.__FIFO_ACCESS)
                                self.__send_data(tx_data)
                                self.__tx_enable()
                                self.__state = self.__STATE_SENDING
                                debug_log("Sending triggered, tx enabled")
                            else:
                                debug_log("Maximum length exceeded could not send")
                        except Exception as e:
                            debug_log("Event but no data in queue! Exception: ", e)
                
            else:
                u'''Unknown event clear event.'''
                event = 0
                debug_log("Unknown State!")
                  
if __name__ == '__main__':
    radio = Rfm22()
    radio.initialise()
    radio.start()
    u'''Wait some time to let the radio module time
    to initialise.'''
    time.sleep(1)
    
    
    u'''Send some data via RFM22. '''
    data = [10,20,30,40,50,60,70,80,90,100]
    for i in range(10):
        time.sleep(0.2)
        radio.put_tx_data(data, block=False)
        print ("put to queue")
        
    u'''Receive data from RFM22'''
    #while(True):
        #data = radio.get_rx_data(block = True)
        #print "RX Data", data, "\n"
        
         
    
    u'''Join RFM22 thread.'''
    radio.join()
    
    

