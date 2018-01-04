/** XDRIP GSM Bridge:
    Project Parakeet

  == Description ==
  The app uses the radio_queue libray to receive packets.  It does not
  transmit any packets.

  Received packets are forwarded via a SIM800 GSM modem to a remote udp or web service.

  This allows for reception in to xDrip remotely using GPRS wide area networking

  UDP payload size is only 8 bytes for basic mode or 16 bytes when including geo location.

  Falls back to HTTP mode if UDP fails.

  GSM modem facilities added by jamorham

  based on wixel-xdrip and xbridge2

  Web service url takes cgi get parameters:

  rr = current ms clock of wixel
  lv = raw value
  lf = filtered value
  ts = ms since packet received
  bp = battery "percent"
  bm = battery millivolts
  ct = cpu temperature, eg 287 = 28.7c
  gl = geo location longitude/latitude

  populate the defines or my_transmitter_id.h file with your local web service
  gprs apn etc

  responds to text message commands:

  STATUS -> reply with status information
  BIGSTATUS -> reply with lots of status information
  REBOOT -> Reboot
  DEFAULTS -> Clear custom configuration
  APN -> Set or Query gprs access point name, eg APN my.carrier
  UDP -> Set UDP server IP or hostname and port, eg UDP 10.123.123.123 21000
  HTTP -> Set Fallback HTTP url, eg HTTP example.com/receiver.cgi
  TRANSMIT -> Set transmitter ID, eg TRANSMIT AAAAA (use this command last)
  LOCK -> Display Lock Status
  LOCK NOW -> Lock to controlling number - commands will then ONLY be accepted from this number
  UNLOCK -> Release Lock

  More documentation at https://jamorham.github.io/


  == Parameters ==
radio_channel: See description in radio_link.h.
*/


/* Features */
// comment out defines below to disable/enable features

#define USE_GEO_LOCATION
#define GET_BATTERY_STATUS
#define USE_UDP_UPLINK
#define USE_HTTP_UPLINK
#define USE_SMS_CONTROL

#define GSM_USE_DTR_PIN

//#define DEBUG

/* Conditional Code */

#ifdef USE_GEO_LOCATION
#define UDP_PACKET_SIZE 16
#define UDP_PACKET_SIZE_STRING "16"
#else
#define UDP_PACKET_SIZE 8
#define UDP_PACKET_SIZE_STRING "8"
#endif

/** Dependencies **************************************************************/
#include <cc2511_map.h>
#include <board.h>
#include <random.h>
#include <time.h>
#include <usb.h>
#include <usb_com.h>
#include <radio_registers.h>
#include <radio_queue.h>
#include <gpio.h>
#include <uart1.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <adc.h>
#include <limits.h>

/* Function prototypes ********************************************************/

void nicersleep (int secs);
void nicedelayMs (int ms);
int gsm_do_sequence ();
void gsm_interactive ();
void gsm_uplink ();
void usb_printf (const char *format, ...);
void gsm_wake_serial ();
int gsm_sleep_mode ();
int gsm_send_sms();
void killWithWatchdog ();
void loadSettingsFromFlash();


void setADCInputs ();


#define myEOF -1
#define CODE_HTTP_UPLINK 99
#define CODE_BATTERY_STATUS 98
#define CODE_UDP_UPLINK 97
#define CODE_GEO_LOCATION 96
#define CODE_SMS_COMMAND 94
#define CODE_APN_POPULATE 93
#define CODE_UDP_POPULATE 92
#define CODE_UDP_CHECK 91
#define CODE_WEB_REPLY 90

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  //
//                           SET THESE VARIABLES TO MEET YOUR NEEDS                                 //
//                                   1 = TRUE       0 = FALSE                                       //
//                                                                                                  //

#ifndef CUSTOM_TRANSMITTER_ID
#warning "Using built-in transmitter id and defines from dexdrip.c"

static CODE const char transmitter_id[] = "ABCDE";                                               //

#define my_webservice_url	"parakeet-receiver.appspot.com/receiver.cgi"
#define my_webservice_reply     "!ACK"
#define my_user_agent 		"xDrip"
#define my_gprs_apn		"apn.check.your.carrier.info"

#define my_udp_server_host	"disabled"
#define my_udp_server_port	"12345"

#define my_control_number       ""

#else
// get user specific configuration from an external untracked file
#include "my_transmitter_id.h"
#endif



//                                                                                                  //
static volatile BIT only_listen_for_my_transmitter = 1;	//
// 1 is recommended                                                                                 //
//                                                                                                  //
static volatile BIT status_lights = 1;	//
// if status_lights = 1; the yellow light flashes while actively scanning                           //
// if a light is flashing for more than 10 minutes straight, it may not be picking up your dex      //
//                                                                                                  //
static volatile BIT allow_alternate_usb_protocol = 1;
// if set to 1 and plugged in to USB then protocol output is suitable for dexterity and similar

// set use_gsm to 0 if you want to use parakeet codebase as USB/Wifi Wixel
static volatile BIT use_gsm = 1;
//                                                                                                  //
//..................................................................................................//
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                  //
//                 Advanced Options, dont change unless you know what you are doing                 //
//                                   1 = TRUE       0 = FALSE                                       //
//                                                                                                  //
//                                                                                                  //
static volatile uint8 wake_earlier_for_next_miss = 20;	//
// if a packet is missed, wake this many seconds earlier to try and get the next one                //
// shorter means better bettery life but more likely to miss multiple packets in a row              //
//                                                                                                  //
static volatile uint8 misses_until_failure = 2;	//
// after how many missed packets should we just start a nonstop scan?                               //
// a high value is better for conserving batter life if you go out of wixel range a lot             //
// but it could also mean missing packets for MUCH longer periods of time                           //
// a value of zero is best if you dont care at all about battery life                               //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////



static XDATA volatile int start_channel = 0;
uint32 XDATA asciiToDexcomSrc (char *addr);
uint32 XDATA getSrcValue (char srcVal);
volatile uint32 dex_tx_id;
#define NUM_CHANNELS        (4)
static int8 fOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
static XDATA int8 defaultfOffset[NUM_CHANNELS] = { 0xCE, 0xD5, 0xE6, 0xE5 };
static uint8 nChannels[NUM_CHANNELS] = { 0, 100, 199, 209 };
static XDATA uint32 waitTimes[NUM_CHANNELS] = { 13500, 500, 500, 500 };
//Now lets try to crank down the channel 1 wait time, if we can 5000 works but it wont catch channel 4 ever
static XDATA uint32 delayedWaitTimes[NUM_CHANNELS] = { 0, 700, 700, 700 };
static XDATA uint32 catch_offsets[NUM_CHANNELS] = { 0, 0, 0, 0 };

static uint8 wake_error_count = 0;
static uint8 last_catch_channel = 0;
BIT needsTimingCalibration = 1;
BIT usbEnabled = 1;
BIT writing_flash = 0;
static volatile BIT send_to_uart = 1;	// when doing usb printf - this is not a setting it is dynamic var
static uint8 save_IEN0;
static uint8 save_IEN1;
static uint8 save_IEN2;

static uint8 XDATA last_dex_battery = 0;
unsigned char XDATA PM2_BUF[7] = { 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x04 };
unsigned char XDATA dmaDesc[8] =
{ 0x00, 0x00, 0xDF, 0xBE, 0x00, 0x07, 0x20, 0x42 };

static volatile BIT usb_connected = 0;
static volatile BIT got_an_error = 0;
static volatile BIT doing_retry = 0;

static uint32 XDATA gsm_delay = 0;	// how long we spent uplinking
static uint32 XDATA lastraw = 0;
static uint32 XDATA lastfiltered = 0;
static uint8 XDATA last_transmittion_id = 0;
static uint32 XDATA lasttime = 0;
static uint32 XDATA general_timer = 0;
volatile uint8 sequential_missed_packets = 0;
static char XDATA lastLocation[30];
static char XDATA messageBuffer[160];
static char XDATA captureBuffer[250];
static char XDATA composeBuffer[250];
static char XDATA stringBuffer[250];
static char XDATA minus[2]="-";

static const char CODE error_string[] = "ERROR";
static const char CODE termMessage[] = { 0x1a, 0x0a, 0x00 };

static uint8  XDATA batteryPercent = 0;
static uint8  XDATA batteryCharging;
#ifdef GET_BATTERY_STATUS
static uint16 XDATA batteryMillivolts = 0;
#endif

#ifdef USE_GEO_LOCATION
static uint32 XDATA longitudeMinor;
static uint32 XDATA latitudeMinor;
static int16  XDATA longitudeMajor;
static int8   XDATA latitudeMajor;
#endif

typedef struct _parakeet_settings
{
    uint32 dex_tx_id; 		//4 bytes
    char http_url[56];
    char gsm_lock[16];
    char gsm_apn[32];
    char udp_server[28];
    char udp_port[6];
    int locked:1;
    int deepsleep:1;
    int pin:13;
    uint8 padding3;
    uint32 checksum; // needs to be aligned

} parakeet_settings;

parakeet_settings XDATA settings;
int XDATA loop;
unsigned XDATA char* flash_pointer;
long unsigned int chk;

#define FLASH_SETTINGS 			(0x7760)

#ifdef USE_SMS_CONTROL
static XDATA char* sfind;
static XDATA char* cfind;
static XDATA char* nfind;

static XDATA char dynamic_transmitter_id[6];

// writeBuffer holds the data from the computer that we want to write in to flash.
XDATA uint8 writeBuffer[sizeof(settings)];

// flashWriteDmaConfig holds the configuration of DMA channel 0, which we use to
// transfer the data from writeBuffer in to flash.
XDATA DMA_CONFIG flashWriteDmaConfig;

// startFlashWrite is a small piece of code we store in RAM which initiates the flash write.
// According to datasheet section 12.3.2.1, this code needs to be 2-aligned if it is executed
// from flash. SDCC does not have a good way of 2-aligning code, so we choose to put the code in
// RAM instead of flash.
XDATA uint8 startFlashWrite[] =
{
    0x75, 0xAE, 0x02,  // mov _FCTL, #2 :   Sets FCTRL.WRITE bit to 1, initiating a write to flash.
    0x22               // ret           :   Returns to the calling function.
};


#endif

typedef struct _Dexcom_packet
{
    uint8 len;
    uint32 dest_addr;
    uint32 src_addr;
    uint8 port;
    uint8 device_info;
    uint8 txId;
    uint16 raw;
    uint16 filtered;
    uint8 battery;
    uint8 unknown;
    uint8 checksum;
    int8 RSSI;
    uint8 LQI;
} Dexcom_packet;

XDATA   int gret = 0;

// Library Routine

uint32 checksum()
{
    chk = 0x12345678;
    flash_pointer = (__xdata unsigned char*)settings;
    for (loop = 0; loop < sizeof(parakeet_settings)-4; loop++)
    {
        chk += (flash_pointer[loop] * (loop + 1));
        chk++;
    }
    return chk;
}

void
strupr (char XDATA *str,char term)
{

    while ((*str) && (*str != term))
    {
        *str = toupper (*str);
        ++str;
    }

}

XDATA char *
xdatstrchr (char XDATA *string, char ch)
{
    while (*string && *string != ch)
        ++string;

    if (*string == ch)
        return string;

    return NULL;
}

XDATA char *
xdatstrstr (char XDATA *str1, char *str2)
{
    XDATA char *cp = str1;
    XDATA char *s1;
    char *s2;


    if (!*str2)
        return str1;

    while (*cp)
    {
        s1 = cp;
        s2 = str2;

        while (*s1 && *s2 && !(*s1-*s2))
            s1++, s2++;

        if (!*s2)
            return cp;

        ++cp;
    }

    return NULL;
}

long
strtol(char *nptr,char **endptr, int base)
__reentrant
{

    char *s = nptr;
    uint32 acc;
    int c;
    unsigned long cutoff;
    int neg = 0, any, cutlim;

    /*
     * Skip white space and pick up leading +/- sign if any.
     * If base is 0, allow 0x for hex and 0 for octal, else
     * assume decimal; if base is already 16, allow 0x.
     */
    do
    {
        c = *s++;
    }
    while (isspace(c));
    if (c == '-')
    {
        neg = 1;
        c = *s++;
    }
    else if (c == '+')
        c = *s++;
    if ((base == 0 || base == 16) &&
    c == '0' && (*s == 'x' || *s == 'X'))
    {
        c = s[1];
        s += 2;
        base = 16;
    }
    else if ((base == 0 || base == 2) &&
    c == '0' && (*s == 'b' || *s == 'B'))
    {
        c = s[1];
        s += 2;
        base = 2;
    }
    if (base == 0)
        base = c == '0' ? 8 : 10;

    cutoff = neg ? -(unsigned long)LONG_MIN : LONG_MAX;
    cutlim = cutoff % (unsigned long)base;
    cutoff /= (unsigned long)base;
    for (acc = 0, any = 0;; c = *s++)
    {
        if (isdigit(c))
            c -= '0';
        else if (isalpha(c))
            c -= isupper(c) ? 'A' - 10 : 'a' - 10;
        else
            break;
        if (c >= base)
            break;
        if (any < 0 || acc > cutoff || acc == cutoff && c > cutlim)
            //if (any < 0)
        {
            any = -1;
        }
        else
        {
            any = 1;
            acc *= base;
            acc += c;
        }
    }


    if (any < 0)
    {
        acc = neg ? LONG_MIN : LONG_MAX;
    }
    else if (neg)
        acc = -acc;
    if (endptr != 0)
        *endptr = (char *)(any ? s - 1 : nptr);
    return (acc);
}

char *scan_string (char *str, XDATA int base)
{
    char *str_ptr = (char*) str;

    switch (base)
    {
    case 10:
        while (!(isdigit(*str_ptr) || *str_ptr == '-' || *str_ptr == 0x0))
        {
            str_ptr++;
        }
        break;
    case 16:
        while (!(isxdigit(*str_ptr) || *str_ptr == 0x0))
        {
            str_ptr++;
        }
        break;
    }

    return str_ptr;
}

int sscanf(char *str, char *fmt, ...)
{
    va_list ap;
    char *format_ptr = (char*)fmt;
    char *str_ptr = (char*)str;

    int8 *p_byte;
    int *p_int;
    long *p_long;

    va_start (ap, fmt);

    while ((*format_ptr != 0x0) && (*str_ptr != 0x0))
    {
        if (*format_ptr == '%')
        {
            format_ptr++;

            if (*format_ptr != 0x0)
            {
                switch (*format_ptr)
                {
                case 'h':       // expect a byte
                    p_byte = va_arg( ap, uint8 *);
                    str_ptr=scan_string(str_ptr, 10);
                    if (*str_ptr==0x0) goto end_parse;
                    *p_byte = (uint8)strtol (str_ptr, &str_ptr, 10);
                    gret ++;
                    break;
                case 'd':       // expect an int
                case 'i':
                    p_int = va_arg( ap, int *);
                    str_ptr=scan_string(str_ptr, 10);
                    if (*str_ptr==0x0) goto end_parse;
                    *p_int = (int)strtol (str_ptr, &str_ptr, 10);
                    gret ++;
                    break;
                case 'D':
                case 'I':       // expect a long
                    p_long = va_arg( ap, long *);
                    str_ptr=scan_string(str_ptr, 10);
                    if (*str_ptr==0x0) goto end_parse;
                    *p_long = strtol (str_ptr, &str_ptr, 10);
                    gret ++;
                    break;
                case 'x':       // expect an int in hexadecimal format
                    p_int = va_arg( ap, int *);
                    str_ptr=scan_string(str_ptr, 16);
                    if (*str_ptr==0x0) goto end_parse;
                    *p_int = (int)strtol (str_ptr, &str_ptr, 16);
                    gret ++;
                    break;
                case 'X':  // expect a long in hexadecimal format
                    p_long = va_arg( ap, long *);
                    str_ptr=scan_string(str_ptr, 16);
                    if (*str_ptr==0x0) goto end_parse;
                    *p_long = strtol (str_ptr, &str_ptr, 16);
                    gret ++;
                    break;
                }
            }
        }

        format_ptr++;
    }

end_parse:
    va_end (ap);

    if (*str_ptr == 0x0) gret = myEOF;
    return gret;
}

#ifdef USE_SMS_CONTROL
//////// from xBridge2 by jstevensog

/* Flash memory functions and variables.
All of these are to do with erasing and writing data into Flash memory.  Used to
store the settings of xBridge.
*/
void dma_Init()
__reentrant
{
    // Configure the flash write timer.  See section 12.3.5 of the datasheet.
    FWT = 32;

    // Set up the DMA configuration block we use for writing to flash (See Figure 21 of the datasheet).
    // LENL and LENH are sent later when we know how much data to write.
    flashWriteDmaConfig.SRCADDRH = (unsigned short)&writeBuffer >> 8;
    flashWriteDmaConfig.SRCADDRL = (unsigned char)&writeBuffer;
    flashWriteDmaConfig.DESTADDRH = XDATA_SFR_ADDRESS(FWDATA) >> 8;
    flashWriteDmaConfig.DESTADDRL = XDATA_SFR_ADDRESS(FWDATA);

    // WORDSIZE = 0     : byte
    // TMODE = 0        : single mode
    // TRIG = 0d18      : flash
    flashWriteDmaConfig.DC6 = 18;

    // SRCINC = 01      : yes
    // DESTINC = 00     : no
    // IRQMASK = 0      : no   *datasheet said this bit should be 1
    // M8 = 0           : 8-bit transfer
    // PRIORITY = 0b10  : high
    flashWriteDmaConfig.DC7 = 0b01000010;

    DMA0CFG = (uint16)&flashWriteDmaConfig;
}
/* eraseFlash:	A function to erase a page of flash.
Note:  This erases a full page of flash, not just the data at the address you specify.
		It works out which page the address you speicfy is in, and erases that page.
		Flash MUST be erased (set every bit in every byte of the flasn page to 1) before
		you can change the data by writing to it.
Parameters:
	uint16	address		Address of the data that you want erased.  Read the note above.
Returns:	void
Uses:	The DMA channel initialised previously.
*/
void eraseFlash(uint16 address)
{
    // first erase the page
    FADDRH  = address >> 9;	// high byte of address / 2
    FADDRL =0;
    FCTL = 1;				// Set FCTL.ERASE to 1 to initiate the erasing.
    __asm nop __endasm;		// Datasheet says a NOP is necessary after the instruction that initiates the erase.
    __asm nop __endasm;		// We have extra NOPs to be safe.
    __asm nop __endasm;
    __asm nop __endasm;
    while(FCTL & 0x80) {};	// Wait for erasing to be complete.
}

/* writeToFlash:	A function to write a value into flash.
Note:  This writes writebuffer to the specified flash address.
Parameters:
	uint16	address		Address of the data that you want erased.  Read the note above.
	uint16	length		The length of the data to write.  Basically the amount of data in writeBuffer.
Returns:	void
Uses:	The DMA channel initialised previously.
	global writeBuffer	Stores the data to be written to flash.
*/
void writeToFlash(uint16 address, uint16 length)
{
    FADDR = address >> 1;	// not sure if i need to do this again.
    flashWriteDmaConfig.VLEN_LENH = length >> 8;
    flashWriteDmaConfig.LENL = length;
    DMAIRQ &= ~(1<<0);		// Clear DMAIF0 so we can poll it to see when the transfer finishes.
    DMAARM |= (1<<0);
    __asm lcall _startFlashWrite __endasm;
    while(!(DMAIRQ & (1<<0))) {}	// wait for the transfer to finish by polling DMAIF0
    while(FCTL & 0xC0) {}		// wait for last word to finish writing by polling BUSY and SWBUSY
}

// Function to save all settings to flash
void saveSettingsToFlash()
{
    writing_flash=1;
    dma_Init();
    settings.checksum=checksum();
#ifdef DEBUG
    usb_printf("BEFORE Setting txid: %lu\r\n",settings.dex_tx_id);
#endif
    usb_printf("Save flash size: %d to location %x\r\n",sizeof(settings),FLASH_SETTINGS);
    memcpy(&writeBuffer, &settings, sizeof(settings));
    eraseFlash(FLASH_SETTINGS);
    writeToFlash(FLASH_SETTINGS, sizeof(settings));
    writing_flash=0;
    usb_printf("settings saved to flash\r\n");
    loadSettingsFromFlash();
    nicedelayMs(10000);
}

#endif

void clearSettings()
{
    memset (&settings, 0, sizeof (settings));
    settings.dex_tx_id = asciiToDexcomSrc (transmitter_id);
    dex_tx_id = settings.dex_tx_id;
    sprintf(settings.http_url,my_webservice_url);
    sprintf(settings.gsm_apn,my_gprs_apn);
    sprintf(settings.udp_server,my_udp_server_host);
    sprintf(settings.udp_port,my_udp_server_port);
    sprintf(settings.gsm_lock,my_control_number);
}

void loadSettingsFromFlash()
{
    memcpy(&settings, (uint8 XDATA *)FLASH_SETTINGS, sizeof(settings));
#ifdef DEBUG
    usb_printf("AFTER Setting txid: %lu\r\n",settings.dex_tx_id);
#endif

    checksum();
#ifdef DEBUG
    usb_printf("Load Settings: tx: %lu / chk calcuated: %lu vs stored %lu\r\n",settings.dex_tx_id,chk,settings.checksum);
#endif
    nicedelayMs(500);
    if (chk!=settings.checksum)
    {
        clearSettings();

        // If custom transmitter_id has been compiled in then we don't wait for initial configuration
        if (dex_tx_id != 10858926)   // ABCDE
        {
            settings.deepsleep=1;
        }
    }
    else
    {
        dex_tx_id = settings.dex_tx_id;
    }
}


int16 getCpuDegC()
{
    return ((((((int32)adcRead(14|ADC_REFERENCE_INTERNAL)*1250 +1023)/2047)-750)*1000)/243);
}

////////

void
sleepInit (void)
{
    WORIRQ |= (1 << 4);
}

ISR (ST, 1)
{
    IRCON &= 0x7F;
    WORIRQ &= 0xFE;
    SLEEP &= 0xFC;
}

void
switchToRCOSC (void)
{
    SLEEP &= ~0x04;
    while (!(SLEEP & 0x20));
    CLKCON = (CLKCON & ~0x07) | 0x40 | 0x01;
    while (!(CLKCON & 0x40));
    SLEEP |= 0x04;
}

void
uartEnable ()
{
    U1UCR &= ~0x40;		//CTS/RTS Off // always off!
    U1CSR |= 0x40;		// Recevier enable
    delayMs (100);
}

void
uartDisable ()
{
    delayMs (100);
    U1UCR &= ~0x40;		//CTS/RTS Off
    U1CSR &= ~0x40;		// Recevier disable
}

void
blink_yellow_led ()
{
    if (status_lights)
    {
        LED_YELLOW (((getMs () / 250) % 2));	//Blink quarter seconds
    }
}

void
blink_red_led ()
{
    if (status_lights)
    {
        LED_RED (((getMs () / 250) % 2));	//Blink 1/4 seconds
    }
}

int8
getPacketRSSI (Dexcom_packet * p)
{
    return (p->RSSI / 2) - 73;
}

uint8
getPacketPassedChecksum (Dexcom_packet * p)
{
    return ((p->LQI & 0x80) == 0x80) ? 1 : 0;
}

uint8
bit_reverse_byte (uint8 in)
{
    uint8 XDATA bRet = 0;
    if (in & 0x01)
        bRet |= 0x80;
    if (in & 0x02)
        bRet |= 0x40;
    if (in & 0x04)
        bRet |= 0x20;
    if (in & 0x08)
        bRet |= 0x10;
    if (in & 0x10)
        bRet |= 0x08;
    if (in & 0x20)
        bRet |= 0x04;
    if (in & 0x40)
        bRet |= 0x02;
    if (in & 0x80)
        bRet |= 0x01;
    return bRet;
}

uint8
min8 (uint8 a, uint8 b)
{
    if (a < b)
        return a;
    return b;
}

void
bit_reverse_bytes (uint8 * buf, uint8 nLen)
{
    uint8 DATA i = 0;
    for (; i < nLen; i++)
    {
        buf[i] = bit_reverse_byte (buf[i]);
    }
}

uint32
dex_num_decoder (uint16 usShortFloat)
{
    uint16 DATA usReversed = usShortFloat;
    uint8 DATA usExponent = 0;
    uint32 DATA usMantissa = 0;
    bit_reverse_bytes ((uint8 *) & usReversed, 2);
    usExponent = ((usReversed & 0xE000) >> 13);
    usMantissa = (usReversed & 0x1FFF);
    return usMantissa << usExponent;
}

char XDATA SrcNameTable[32] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
                                'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P',
                                'Q', 'R', 'S', 'T', 'U', 'W', 'X', 'Y'
                              };

#ifdef USE_SMS_CONTROL

void
dexcom_src_to_ascii ()
{
    dynamic_transmitter_id[0] = SrcNameTable[(dex_tx_id >> 20) & 0x1F];
    dynamic_transmitter_id[1] = SrcNameTable[(dex_tx_id >> 15) & 0x1F];
    dynamic_transmitter_id[2] = SrcNameTable[(dex_tx_id >> 10) & 0x1F];
    dynamic_transmitter_id[3] = SrcNameTable[(dex_tx_id >> 5) & 0x1F];
    dynamic_transmitter_id[4] = SrcNameTable[(dex_tx_id >> 0) & 0x1F];
    dynamic_transmitter_id[5] = 0;
}

#endif

void
doServices ()
{
    if (usb_connected)
        //if (usbPowerPresent ())
    {
        usbComService ();
        boardService ();
    }
}

void
initUart1 ()
{
    uart1Init ();
    uart1SetBaudRate (9600);
}


// you can uncomment this if you want a glowing yellow LED when a terminal program is connected
// to the USB.  I got sick of it.
// LineStateChangeCallback - sets the yellow LED to the state of DTR on the USB, whenever it changes.
//void
//LineStateChangeCallback (uint8 state)
//{
//    //LED_YELLOW(state & ACM_CONTROL_LINE_DTR);
//    usb_connected = state & ACM_CONTROL_LINE_DTR;
//}

uint32
asciiToDexcomSrc (char addr[6])
{
    uint32 XDATA src = 0;
    src |= (getSrcValue (addr[0]) << 20);
    src |= (getSrcValue (addr[1]) << 15);
    src |= (getSrcValue (addr[2]) << 10);
    src |= (getSrcValue (addr[3]) << 5);
    src |= getSrcValue (addr[4]);
    return src;
}

uint32
getSrcValue (char srcVal)
{
    uint8 i = 0;
    for (i = 0; i < 32; i++)
    {
        if (SrcNameTable[i] == srcVal)
            break;
    }
    return i & 0xFF;
}



///////// JAMORHAM GPRS / GSM MODEM FEATURE /////////////////


typedef struct _Serial_cmd
{
    const char *command;
    const char *response;
    const int timeout;
    const int getdata;
} Serial_cmd;

CODE Serial_cmd *gsm_cmd = full_gsm_cmd;

#ifdef USE_SMS_CONTROL
static const Serial_cmd CODE sms_cmd[] =
{
    {"ATZ", "OK", 10},
    {"ATE0","OK", 2},
    {"AT+CSCLK=1", "OK", 2},
    {"AT+CFUN=1", "OK", 30},
    {"AT+CMGF=1","OK",2},
#ifdef GET_BATTERY_STATUS
    {"AT+CBC","+CBC: ",2,CODE_BATTERY_STATUS},
#endif
    {"AT+CMGL=\"REC UNREAD\"","OK",5,CODE_SMS_COMMAND},
    {"AT+CMGDA=\"DEL READ\"","OK",5},
    {"AT+CMGDA=\"DEL SENT\"","OK",5},
// end of sequence marker
    {"END", "END", 255}
};
#endif


const Serial_cmd CODE full_gsm_cmd[] =
{
    {"ATZ", "OK", 10},
    {"ATE0","OK", 2},
    {"AT+CFUN=0", "", 10},
    {"AT+CSCLK=1", "OK", 2},
    {"AT+CFUN=1", "Call Ready", 30},

#ifdef GET_BATTERY_STATUS
    {"AT+CBC","+CBC: ",2,CODE_BATTERY_STATUS},
#endif

    {"AT+SAPBR=0,1", "", 3},
    {"AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2},
    {"AT+SAPBR=3,1,\"APN\",\"%s\"", "OK", 2,CODE_APN_POPULATE},
    {"AT+SAPBR=1,1", "OK", 30},

#ifdef USE_GEO_LOCATION
    {"AT+CIPGSMLOC=1,1","+CIPGSMLOC: ",15,CODE_GEO_LOCATION},
#endif

#ifdef USE_SMS_CONTROL
    {"AT+CMGF=1","OK",2},
    {"AT+CMGL=\"REC UNREAD\"","OK",5,CODE_SMS_COMMAND},
    {"AT+CMGDA=\"DEL READ\"","OK",5},
    {"AT+CMGDA=\"DEL SENT\"","OK",5},
#endif

#ifdef USE_UDP_UPLINK
    {"UDP_CHECK","",11,CODE_UDP_CHECK},
    {"AT+CIPSHUT","SHUT OK",5},
    {"AT+CGATT?","CGATT: 1",2},
    {"AT+CIPCSGP=1,\"%s\"","OK",2,CODE_APN_POPULATE},
    {"AT+CSTT=\"%s\"","OK",2,CODE_APN_POPULATE},
    {"AT+CIICR","OK",10},
    {"AT+CIPSTATUS","IP GPRSACT",4},
    {"AT+CIFSR",".",10},
    {"AT+CIPSTART=\"UDP\",\"%s\"","OK",10,CODE_UDP_POPULATE},
    {"AT+CIPSEND=" UDP_PACKET_SIZE_STRING,">",5},
    {"UDP","~",10,CODE_UDP_UPLINK},
    {"AT+CIPSHUT","SHUT OK",5},

#endif

#ifdef USE_HTTP_UPLINK
    {"AT+HTTPTERM", "", 2},
    {"AT+HTTPINIT", "", 10},
    {"AT+HTTPPARA=\"CID\",1", "OK", 2},
    {
        "AT+HTTPPARA=\"URL\",\"%s%s\"", "OK",
        2,CODE_HTTP_UPLINK
    },
    {"AT+HTTPPARA=\"UA\",\"" my_user_agent "\"", "OK", 2},
    {"AT+HTTPACTION=0", "+HTTPACTION: 0,200,", 60},
    {"AT+HTTPREAD", my_webservice_reply , 2, CODE_WEB_REPLY},
    {"AT+HTTPTERM", "OK", 2},
#endif

// end of sequence marker
    {"END", "END", 255}
};


#ifdef USE_SMS_CONTROL
void sendStatus()
{
    dexcom_src_to_ascii ();
    sprintf(messageBuffer,"STAT: id:%s batt:%d raw:%lu filt:%lu https://maps.google.com/?q=%s",dynamic_transmitter_id,batteryPercent,lastraw,lastfiltered,lastLocation);
    gsm_send_sms();
}

int matchCommand(const char* command)
{
    int len = strlen(command);
    if (strncmp(command,sfind,len)==0)
    {
        usb_printf("Matched COMMAND: %s\r\n",command);
        return 1;
    }
    else
    {
        return 0;
    }
}

void sendAPN()
{
    sprintf(messageBuffer,"APN: '%s'",settings.gsm_apn);
    gsm_send_sms();
}
void sendLock()
{
    sprintf(messageBuffer,"Locked to number: '%s'",settings.gsm_lock);
    gsm_send_sms();
}
void sendUDP()
{
    sprintf(messageBuffer,"UDP: '%s:%s'",settings.udp_server,settings.udp_port);
    gsm_send_sms();
}
void sendHTTP()
{
    sprintf(messageBuffer,"HTTP: '%s'",settings.http_url);
    gsm_send_sms();
}

#endif


int
gsm_get_response (const char *response, int timeout, int getdata)
{
    BIT looking_for_ok = 0;
    char b = 0;
    int rolling_ptr = 0;
    static int error_ptr = 0;
    int len = strlen (response);
    uint32 timeout_time = getMs () + ((uint32) timeout * 1000);
    loop = 0; // move ptr to global
    if ((got_an_error == 0) && (strcmp("OK",response)==0))
    {
        looking_for_ok = 1;
    }
    while ((loop < len) && (getMs () < timeout_time))
    {
        if (uart1RxAvailable ())
        {
            b = uart1RxReceiveByte ();
            if (usb_connected)
                usbComTxSendByte (b);
            if (getdata == CODE_SMS_COMMAND)
            {
                captureBuffer[rolling_ptr]=b;
                rolling_ptr++;
                if (rolling_ptr>(sizeof(captureBuffer)-2))
                {
                    rolling_ptr--;
                }
            }
            if (response[loop] == b)
            {
                loop++;
            }
            else
            {
                loop = 0;
            }
            if (looking_for_ok)
            {
                if (error_string[error_ptr] == b)
                {
                    error_ptr++;
                }
                else
                {
                    error_ptr=0;
                }
                if (error_ptr == (sizeof(error_string)-1))
                {
#ifdef DEBUG
                    usb_printf("Matched ERROR string\r\n");
#endif
                    got_an_error = 1;
                    timeout_time = 0;
                }
            }
        }
        else
        {
            doServices ();
            delayMicroseconds (100);
        }

    }

    if ((len > 0) && (loop < len))
    {
        usb_printf ("Timedout on %s\r\n", response);
        return 0;
    }
    else
    {
#ifdef DEBUG
        usb_printf (" <-- Matched %s\r\n", response);
#endif
        got_an_error = 0; // clear flag on success
#ifdef USE_SMS_CONTROL
        if (getdata == CODE_SMS_COMMAND)
        {
            captureBuffer[rolling_ptr]='\0';
#ifdef DEBUG
            usb_printf ("MULTI LINE capture :%s:\r\n", captureBuffer);
#endif
            sfind = xdatstrstr(captureBuffer,"\",\"");
            if (sfind !=NULL)
            {
                sfind=sfind+3;
                cfind =  xdatstrchr(sfind, '"');
                if (cfind !=NULL)
                {
                    *cfind = '\0';

                    usb_printf ("Isolated phone number :%s:\r\n", sfind);
                    nfind = sfind;
                    sfind = xdatstrchr(++cfind,'\n');
                    if (sfind != NULL)
                    {
                        cfind = xdatstrchr(++sfind,'\r');
                        *cfind = '\0';
                        usb_printf ("Isolated Text Message :%s:\r\n", sfind);

                        if ((settings.gsm_lock[0]=='\0') || (strncmp(settings.gsm_lock,nfind,sizeof(settings.gsm_lock))==0))
                        {

                            strupr(sfind,' ');
                            if (matchCommand("BIGSTATUS"))
                            {
                                sendStatus();
                                sendAPN();
                                sendLock();
                                sendUDP();
                                sendHTTP();
                            }
                            else if (matchCommand("STATUS"))
                            {
                                sendStatus();
                            }
                            else if (matchCommand("TRANSMIT"))
                            {
                                if (strlen(sfind)==14)
                                {
                                    strupr(sfind,'\0');
                                    dex_tx_id = asciiToDexcomSrc (sfind+9);
                                    settings.dex_tx_id = dex_tx_id;
                                    settings.deepsleep = 1;
                                    saveSettingsToFlash();
                                    sendStatus();
                                }
                                else
                                {
                                    sprintf(messageBuffer,"%s","Invalid Transmit command length");
                                    gsm_send_sms();
                                }

                            }
                            else if (matchCommand("REBOOT"))
                            {
                                killWithWatchdog();
                            }
                            else if (matchCommand("DEFAULTS"))
                            {
                                clearSettings();
                                saveSettingsToFlash();
                                killWithWatchdog();
                            }
                            else if (matchCommand("APN"))
                            {
                                if (strlen(sfind)>4)
                                {
                                    memcpy(&settings.gsm_apn, sfind+4, (sizeof(settings.gsm_apn)-1));
                                    saveSettingsToFlash();
                                }
                                sendAPN();
                            }
                            else if (matchCommand("LOCK"))
                            {
                                if (strlen(sfind)>5)
                                {
                                    memcpy(&settings.gsm_lock,nfind,(sizeof(settings.gsm_lock)-1));
                                    saveSettingsToFlash();
                                }
                                sendLock();
                            }
                            else if (matchCommand("UNLOCK"))
                            {
                                settings.gsm_lock[0]='\0';
                                saveSettingsToFlash();
                                sendLock();
                            }
                            else if (matchCommand("UDP"))
                            {
                                if (strlen(sfind)>11)
                                {
                                    sfind=sfind+4;
                                    cfind = xdatstrchr(sfind,' ');
                                    *cfind='\0';
                                    memcpy(&settings.udp_server,sfind,(sizeof(settings.udp_server)-1));
                                    memcpy(&settings.udp_port,++cfind,(sizeof(settings.udp_port)-1));
                                    saveSettingsToFlash();
                                }
                                sendUDP();
                            }
                            else if (matchCommand("HTTP"))
                            {
                                if (strlen(sfind)>11)
                                {
                                    memcpy(&settings.http_url,sfind+5,(sizeof(settings.http_url)-1));
                                    saveSettingsToFlash();
                                }
                                sendHTTP();
                            }

                        }
                        else
                        {
                            usb_printf("Not responding to wrong number when locked\r\n");
                        }

                    }
                }

            }
            return 1;
        }
#endif

read_line_of_data:
        // read in any extra data
        if (getdata>0)
        {
#ifdef DEBUG
            usb_printf ("Getting data reply\r\n");
#endif
            loop=0;
            while ((b != '\r') && (getMs () < timeout_time) && (loop<sizeof(captureBuffer)))
            {
                if (uart1RxAvailable ())
                {
                    b = uart1RxReceiveByte ();
                    if (usb_connected)
                        usbComTxSendByte (b);
                    if (b != '\r')
                    {
                        captureBuffer[loop]=b;
                        loop++;
                    }
                    else
                    {
                        captureBuffer[loop]='\0';
#ifdef DEBUG
                        usb_printf ("Got data reply %s\r\n",captureBuffer);
#endif

                        switch (getdata)
                        {


#ifdef GET_BATTERY_STATUS
                        case CODE_BATTERY_STATUS:
                            sscanf(captureBuffer,"%h,%h,%d",&batteryCharging,&batteryPercent,&batteryMillivolts);
#ifdef DEBUG
                            usb_printf("Battery read result: %hhu / %hhu / %d\r\n",batteryCharging,batteryPercent,batteryMillivolts);
#endif
                            captureBuffer[0]='\0';
                            break;
#endif


#ifdef USE_GEO_LOCATION
                        case CODE_GEO_LOCATION:
                            if (strlen(captureBuffer)>12)
                            {
                                sscanf(&captureBuffer[2],"%d.%D,%h.%D,",&longitudeMajor,&longitudeMinor,&latitudeMajor,&latitudeMinor);
                                if ((longitudeMajor==0)&&(captureBuffer[2]=='-'))
                                {
                                    minus[0]='-';
                                }
                                else
                                {
                                    minus[0]='\0';
                                }
                                sprintf(lastLocation,"%hhd.%ld,%s%d.%ld",latitudeMajor,latitudeMinor,minus,longitudeMajor,longitudeMinor);
                                if ((longitudeMajor==0)&&(captureBuffer[2]=='-')) longitudeMajor=255;
#ifdef DEBUG
                                usb_printf("Geo Location read result: %d %ld / %hhd %ld\r\n",longitudeMajor,longitudeMinor,latitudeMajor,latitudeMinor);
                                usb_printf("Last Location set to: %s\r\n",lastLocation);
#endif
                            }
                            else
                            {
                                usb_printf("Could not get GEO Location: %s\r\n",captureBuffer);
                            }
                            break;
#endif

                        case CODE_WEB_REPLY:
                        {
                            if (strlen(captureBuffer)>3)
                            {
#ifdef DEBUG
                                usb_printf("WEB REPLY BUFFER: %c %s\r\n",captureBuffer[2],captureBuffer[3],captureBuffer);
#endif
                                if (captureBuffer[3]=='!')
                                {
                                    switch (captureBuffer[2])
                                    {
                                    case '2':
#ifdef DEBUG
                                        usb_printf("Stay awake command received\r\n");
#endif
                                        settings.deepsleep = 0;
                                        break;
                                    }
                                }
                            }
                            break;
                        }


                        }

                        return 1;
                    }
                }
            }
            if (getdata>1)
            {
                // if strict
                return 0;
            }
            else
            {
                return 1;
            }
        }
        else
        {
            return 1;
        }
    }				// did we get a match
}				// function end

void
usb_printf (const char *format, ...)
{
    va_list argx;
    if (usb_connected)
    {
        send_to_uart = 0;
        va_start (argx, format);
        vprintf (format, argx);
        send_to_uart = 1;
    }
}

void
print_packet (Dexcom_packet * pPkt)
{

    uint32 mlasttime;
    lasttime = getMs ();
    lastraw = dex_num_decoder (pPkt->raw);
    lastfiltered = dex_num_decoder (pPkt->filtered) * 2;
    last_transmittion_id = (pPkt->txId & 0xFC) >> 2;

    uartEnable ();

    if ((allow_alternate_usb_protocol == 0) || !usbPowerPresent ())
    {

        // Classic 3 field protocol for serial/bluetooth only
        if (!use_gsm)
            printf ("%lu %hhu %d", dex_num_decoder (pPkt->raw), pPkt->battery,
                    adcConvertToMillivolts (adcRead (0)));

    }
    else
    {

        // Protocol suitable for dexterity android application or python script when running in USB mode
        usb_printf ("%lu %lu %lu %hhu %d %hhu %d \r\n", pPkt->src_addr,
                    dex_num_decoder (pPkt->raw),
                    dex_num_decoder (pPkt->filtered) * 2, pPkt->battery,
                    getPacketRSSI (pPkt), pPkt->txId,
                    adcConvertToMillivolts (adcRead (0)));
    }

    gsm_delay = getMs ();
    if (use_gsm)
    {
        last_dex_battery = pPkt->battery;
        gsm_uplink ();		// send data via gsm / gprs

        gsm_delay = (getMs () - gsm_delay) / 1000;

#ifdef USE_SMS_CONTROL
        if (!settings.deepsleep)
        {
            mlasttime = getMs();
            while ((!settings.deepsleep) && (getMs()-mlasttime)<600000)
            {
                gsm_cmd = sms_cmd;
                gsm_do_sequence();
                nicedelayMs(3000);
            }
            gsm_cmd = full_gsm_cmd;
            settings.deepsleep = 1;
            gsm_sleep_mode();
        }
#endif
    }


    uartDisable ();



}

int
gsm_send_command_getdata (const char *command, const char *response, int timeout, XDATA int getdata)
{

    int i;
    char b;
    char XDATA buffer[256];
    char XDATA parambuf[256] = "";
    char* XDATA param = parambuf;

    stringBuffer[0]=0; // clean up

#ifdef USE_HTTP_UPLINK

    if (getdata==CODE_HTTP_UPLINK)
    {
        // HTTP SEND

#ifdef GET_BATTERY_STATUS
        sprintf (param,"%s?rr=%lu&zi=%lu&pc=%s&lv=%lu&lf=%lu&db=%hhu",settings.http_url, getMs (), dex_tx_id, settings.udp_port, lastraw, lastfiltered,last_dex_battery);
        sprintf (stringBuffer, "&ts=%lu&bp=%d&bm=%d&ct=%d&gl=%s&ti=%hhu", (getMs () - lasttime), batteryPercent, batteryMillivolts, getCpuDegC(), lastLocation, last_transmittion_id);
#else
        sprintf (param, "%s?rr=%lu&zi=%lu&pc=%s&lv=%lu", settings.http_url,getMs (), dex_tx_id, settings.udp_port lastraw,
                 sprintf (stringBuffer,"&lf=%lu&ts=%lu&ct=%d&gl=%s",
                          lastfiltered, (getMs () - lasttime), getCpuDegC(),lastLocation);
#endif
        getdata=0;
    }
#endif


// read in any buffered data before we send the command

    while (uart1RxAvailable ())
    {
        b = uart1RxReceiveByte ();
        if (usb_connected)
            usbComTxSendByte (b);
    }

#ifdef USE_UDP_UPLINK
    if (getdata==CODE_UDP_UPLINK)
    {

        buffer[0]=(char)((lastraw >> 16) & 0xff);
        buffer[1]=(char)((lastraw >> 8) & 0xff);
        buffer[2]=(char)(lastraw & 0xff);

        buffer[3]=(char)((lastfiltered >> 16) & 0xff);
        buffer[4]=(char)((lastfiltered >> 8) & 0xff);
        buffer[5]=(char)(lastfiltered & 0xff);

        buffer[6]=(char)(batteryPercent);
        buffer[7]=(char)((getMs()-lasttime)/1000);

#ifdef USE_GEO_LOCATION
        buffer[8]=(char)latitudeMajor;
        buffer[9]=(char)((latitudeMinor >> 16) & 0xff);
        buffer[10]=(char)((latitudeMinor >> 8) & 0xff);
        buffer[11]=(char)((latitudeMinor) & 0xff);
        buffer[12]=(char)(longitudeMajor >>1) & 0xff;
        buffer[13]=(char)(((longitudeMinor >> 16) & 0xff) | ((longitudeMajor << 7) & 0x80));
        buffer[14]=(char)((longitudeMinor >> 8) & 0xff);
        buffer[15]=(char)((longitudeMinor) & 0xff);
#endif


        for (i=0; i<UDP_PACKET_SIZE; i++)
        {
            uart1TxSendByte (buffer[i]);
        }
        getdata=0;
        goto gsm_send_command_return;
    }
#endif

    if (getdata==CODE_APN_POPULATE)
    {
        param=settings.gsm_apn;
        getdata=0;
    }
    else if (getdata==CODE_UDP_POPULATE)
    {
        sprintf (param, "%s\",\"%s", settings.udp_server,settings.udp_port);
        getdata=0;
    }
    sprintf (buffer, command, param, stringBuffer);	// fill in string

    usb_printf ("GSM send command: %s / %s / %d\r\n", buffer, response,
                timeout);

    printf (buffer);
    printf ("\r\n");		// enter key

gsm_send_command_return:
    return gsm_get_response (response, timeout, getdata);
}

int
gsm_send_command (const char *command, const char *response, int timeout)
{
    return gsm_send_command_getdata(command,response,timeout,0);
}

int
gsm_send_sms()
{
    sprintf(composeBuffer,"AT+CMGS=\"%s\"" ,nfind);
    if (gsm_send_command_getdata(composeBuffer,">",3,0))
    {
        gsm_send_command(messageBuffer,"",2);
        return gsm_send_command_getdata(termMessage,"+CGMS:",10,0);
    }
    else
    {
        return 0;
    }
}

/////////////////////////////


void
makeAllOutputsLow ()
__reentrant
{
    int i;
    for (i = 0; i < 16; i++)
    {
        if (i != 13)
            setDigitalOutput (i, LOW);	// skip P1_3 for dtr line
    }
    if (!use_gsm)
    {
        P0INP = 0;  //set pull resistors on pins 0_0 - 0_5 to low
    }
}

void
reset_offsets ()
{
    int i;
    for (i = 0; i < 4; i++)
    {
        fOffset[i] = defaultfOffset[i];
    }
}

void
killWithWatchdog ()
{
    WDCTL = (WDCTL & ~0x03) | 0x00;
    WDCTL = (WDCTL & ~0x04) | 0x08;
}

void
goToSleep (int32 seconds)
__reentrant
{

    if (use_gsm)
    {
        LED_YELLOW(0); // shut off any indicator led
        seconds = seconds - gsm_delay;
        usb_printf ("Sleep Delay adjusted for gsm by %ld\r\n", gsm_delay);
        gsm_delay = 0;
    }

    if (seconds < 1)
        return;

    adcSetMillivoltCalibration (adcReadVddMillivolts ());
    makeAllOutputsLow ();

    if (!needsTimingCalibration)
    {
        if (!usbPowerPresent ())
        {
            unsigned char temp;
            unsigned char storedDescHigh, storedDescLow;
            BIT storedDma0Armed;
            unsigned char storedIEN0, storedIEN1, storedIEN2;

            uint8 savedPICTL = PICTL;
            BIT savedP0IE = P0IE;
            uint8 savedP0SEL = P0SEL;
            uint8 savedP0DIR = P0DIR;
            uint8 savedP1SEL = P1SEL;
            uint8 savedP1DIR = P1DIR;

            sleepInit ();

            disableUsbPullup ();
            usbDeviceState = USB_STATE_DETACHED;
            usbEnabled = 0;
            SLEEP &= ~(1 << 7);

            WORCTRL |= 0x03;	// 2^5 periods
            switchToRCOSC ();

            storedDescHigh = DMA0CFGH;
            storedDescLow = DMA0CFGL;
            storedDma0Armed = DMAARM & 0x01;
            DMAARM |= 0x81;
            dmaDesc[0] = ((unsigned int) &PM2_BUF) >> 8;
            dmaDesc[1] = (unsigned int) &PM2_BUF;

            DMA0CFGH = ((unsigned int) &dmaDesc) >> 8;
            DMA0CFGL = (unsigned int) &dmaDesc;
            DMAARM = 0x01;

            // save enabled interrupts
            storedIEN0 = IEN0;
            storedIEN1 = IEN1;
            storedIEN2 = IEN2;

            //enable sleep timer interrupt
            IEN0 |= 0xA0;

            //disable all interrupts except the sleep timer
            IEN0 &= 0xA0;
            IEN1 &= ~0x3F;
            IEN2 &= ~0x3F;

            WORCTRL |= 0x04;	// Reset
            temp = WORTIME0;
            while (temp == WORTIME0)
            {
            };
            WOREVT1 = seconds >> 8;
            WOREVT0 = seconds;

            temp = WORTIME0;
            while (temp == WORTIME0)
            {
            };

            MEMCTR |= 0x02;
            SLEEP = 0x06;
            __asm nop __endasm;
            __asm nop __endasm;
            __asm nop __endasm;
            if (SLEEP & 0x03)
            {
                __asm mov 0xD7, #0x01 __endasm;
                __asm nop __endasm;
                __asm orl 0x87, #0x01 __endasm;
                __asm nop __endasm;
            }
            IEN0 = storedIEN0;
            IEN1 = storedIEN1;
            IEN2 = storedIEN2;
            DMA0CFGH = storedDescHigh;
            DMA0CFGL = storedDescLow;
            if (storedDma0Armed)
            {
                DMAARM |= 0x01;
            }
            // Switch back to high speed
            boardClockInit ();

            PICTL = savedPICTL;
            P0IE = savedP0IE;
            P0SEL = savedP0SEL;
            P0DIR = savedP0DIR;
            P1SEL = savedP1SEL;
            P1DIR = savedP1DIR;
            USBPOW = 1;
            USBCIE = 0b0111;
        }
        else
        {
            uint32 start_waiting = getMs ();
            if (!usbEnabled)
            {
                usbDeviceState = USB_STATE_POWERED;
                enableUsbPullup ();
                usbEnabled = 1;
            }
            delayMs (100);
            while ((getMs () - start_waiting) < (seconds * 1000))
            {
                doServices ();
            }
        }
    }
    makeAllOutputsLow ();
}


void
putchar (char c)
{
    if (send_to_uart)
        uart1TxSendByte (c);
    if (usbPowerPresent ())
    {
        while (usbComTxAvailable()<100)
        {
            usbComService();
        }
        usbComTxSendByte (c);
    }
}

void
swap_channel (uint8 channel, uint8 newFSCTRL0)
{
    do
    {
        RFST = 4;			//SIDLE
    }
    while (MARCSTATE != 0x01);

    FSCTRL0 = newFSCTRL0;
    CHANNR = channel;
    RFST = 2;			//RX
}

void
strobe_radio (int radio_chan)
{
    radioMacInit ();
    MCSM1 = 0;
    radioMacStrobe ();
    swap_channel (nChannels[radio_chan], fOffset[radio_chan]);
}

int
WaitForPacket (uint16 milliseconds, Dexcom_packet * pkt, uint8 channel)
__reentrant
{
    uint32 start = getMs ();
    uint8 *packet = 0;
    uint32 i = 0;
#define six_minutes 360000
    int nRet = 0;
    swap_channel (nChannels[channel], fOffset[channel]);

    while (!milliseconds || (getMs () - start) < milliseconds)
    {
        doServices ();
        blink_yellow_led ();
        i++;
        if (!(i % 40000))
        {
            strobe_radio (channel);
        }
        if ((getMs () - start) > six_minutes)
        {
            killWithWatchdog ();
            delayMs (2000);
        }
        if (packet = radioQueueRxCurrentPacket ())
        {
            uint8 len = packet[0];
            fOffset[channel] += FREQEST;
            memcpy (pkt, packet, min8 (len + 2, sizeof (Dexcom_packet)));
            if (radioCrcPassed ())
            {
                if (pkt->src_addr == dex_tx_id || dex_tx_id == 0
                        || only_listen_for_my_transmitter == 0)
                {
                    pkt->txId -= channel;
                    radioQueueRxDoneWithPacket ();
                    LED_YELLOW (0);
                    last_catch_channel = channel;
                    return 1;
                }
                else
                {
                    radioQueueRxDoneWithPacket ();
                }
            }
            else
            {
                radioQueueRxDoneWithPacket ();
                LED_YELLOW (0);
                return 0;
            }
        }
    }
    LED_YELLOW (0);
    return nRet;
}

uint32
delayFor (int wait_chan)
{
    if (needsTimingCalibration)
    {
        return delayedWaitTimes[wait_chan];
    }
    if (!wait_chan && sequential_missed_packets)
    {
        return waitTimes[wait_chan] +
               (sequential_missed_packets * wake_earlier_for_next_miss * 2 * 1000);
    }
    else
    {
        return waitTimes[wait_chan];
    }
}

BIT
get_packet (Dexcom_packet * pPkt)
{
    int nChannel = 0;
    for (nChannel = start_channel; nChannel < NUM_CHANNELS; nChannel++)
    {
        switch (WaitForPacket (delayFor (nChannel), pPkt, nChannel))
        {
        case 1:
            needsTimingCalibration = 0;
            sequential_missed_packets = 0;
            return 1;
        case 0:
            continue;
        }
    }
    needsTimingCalibration = 1;
//    killWithWatchdog();
//   delayMs(2000);
    sequential_missed_packets++;
    if (sequential_missed_packets > misses_until_failure)
    {
        sequential_missed_packets = 0;
        needsTimingCalibration = 1;
    }
    reset_offsets ();
    last_catch_channel = 0;
    return 0;
}


void flashingError(int XDATA howlong,int XDATA flashspeed)
{

#ifdef DEBUG
    usb_printf("Doing error light sequence\r\n");
#endif

    general_timer=getMs()+howlong;
    while (getMs()<general_timer)
    {
        LED_RED (!(((getMs () / flashspeed)) % 2) & 1);
        LED_YELLOW ((((getMs () / flashspeed)) % 2) & 1);
        doServices();
    }
    LED_RED(0); // lights off after error
    LED_YELLOW(0);

    wake_error_count++;
    if (wake_error_count > 2)
    {
        killWithWatchdog ();
    }
}

void
configBt ()
{
    uartEnable ();
    if (use_gsm)
    {
        if  (!gsm_sleep_mode ())
        {
            // problem with init of gsm module
            flashingError(20000,400);
        }

#ifdef DEBUG
        gsm_interactive(); // for debug
#endif

#ifdef USE_SMS_CONTROL
        if (!settings.deepsleep)
        {
            gsm_wake_serial();
            while (!settings.deepsleep)
            {
#ifdef DEBUG
                usb_printf("Checksums: calc %lu / stored = %lu / txid: %lu / USB: %hhu\r\n",chk,settings.checksum,dex_tx_id,usbDeviceState);
                nicedelayMs(300);
#endif
                gsm_cmd = sms_cmd;
                gsm_do_sequence();
                nicedelayMs(3000);
            }
        }
#endif

        gsm_cmd = full_gsm_cmd;

    }
    else
    {
        printf ("AT+NAMExDrip");
    }

    uartDisable ();
}

int
gsm_sleep_mode ()
{
    int XDATA result = 1;
    gsm_wake_serial ();
    if (settings.deepsleep)
    {
        usb_printf ("Sleeping GSM\r\n");
#ifdef GSM_USE_DTR_PIN
        gsm_send_command ("AT+CSCLK=1", "OK", 2);
#else
        gsm_send_command ("ATZ", "OK", 2);
#endif
        if (!gsm_send_command ("AT+CFUN=0", "OK", 10)) result = 0; // pass error
        delayMs (2000);
#ifdef GSM_USE_DTR_PIN
        setDigitalOutput (13, HIGH);	// P1_3 set high to sleep
#endif
    }
    else
    {
        usb_printf("Not DeepSleeping - config unset\r\n");
    }
    return result;
}

void
gsm_wake_serial ()
__reentrant
{
    int i;
    usb_printf ("Waking GSM\r\n");
#ifdef GSM_USE_DTR_PIN
    setDigitalOutput (13, LOW);	// P1_3 set low to wake
#endif
    for (i = 0; i < 10; i++)
    {
        uart1TxSendByte (' ');
    }
    delayMs (200);

    if (!gsm_send_command ("AT", "OK", 1))
    {
        delayMs (200);
        if (!gsm_send_command ("AT", "OK", 1))
        {
            uart1TxSendByte (27); // ESC
            delayMs (300);
            if (!gsm_send_command ("AT", "OK", 2))
            {
                flashingError(20000,200); // double speed flash means could not wake
            }
            else
            {
                wake_error_count = 0;
            }
        }
        else
        {
            wake_error_count = 0;
        }
    }
    else { wake_error_count = 0; }
}

void
gsm_uplink ()
{
    int i = 0;
    int result;
    while ((i < 5) && (i > -1))
    {
        usb_printf ("Attempting to uplink data: try: %d\r\n", i);
        result = gsm_do_sequence ();
        i++;
        if (result)
        {
            i = -10;
        }
        else
        {
            delayMs (3000);
        }
    }
    if (i == -10)
    {
        usb_printf ("UPLINK SUCCESS!\r\n");
    }
    else
    {
        usb_printf ("UPLINK FAILED!\r\n");

    }
}

#ifdef DEBUG

void
gsm_interactive ()
__reentrant
{
    usb_printf ("Entering interactive mode\r\n");
    gsm_wake_serial();
    while (1)
    {

        char b;


        while (uart1RxAvailable ())
        {
            b = uart1RxReceiveByte ();
            if (usb_connected)
                usbComTxSendByte (b);	// send as debug output
        }
        while (usbComRxAvailable ())
        {
            b = usbComRxReceiveByte ();
            if (b == '¬')
                gsm_do_sequence ();
            if (b == '`')
            {
                usb_printf ("exit interactive\r\n");
                return;
            }
            uart1TxSendByte (b);
        }

        doServices ();
        delayMs (10);
    }				// main while

}
#endif

int
gsm_do_sequence ()
{
    int i = 0;
    int result = 1;
    BIT terminate_on_next = 0;

    LED_GREEN (1);
    LED_RED (0);

    captureBuffer[0]='\0';
    doing_retry = 0;

    usb_printf ("Doing GSM sequence\r\n");
    gsm_wake_serial ();
    while (result)
    {

#ifdef USE_UDP_UPLINK
        // Skip UDP protocol if udp server starts with "disable"
        if (gsm_cmd[i].getdata == CODE_UDP_CHECK)
        {
            usb_printf("Skipping UDP");
            if (strncmp("disable",settings.udp_server,7)==0)
            {
                i=i+gsm_cmd[i].timeout;
            }
            else
            {
                i++;
            }
        }
#endif

        if (gsm_cmd[i].timeout == 255)
            break;			// exit loop when on last cmd


        LED_RED (1);
        result =
            gsm_send_command_getdata (gsm_cmd[i].command, gsm_cmd[i].response,
                                      gsm_cmd[i].timeout, gsm_cmd[i].getdata);
        LED_RED (0);

        if (result)
        {
            delayMs (50);
            if (terminate_on_next) break;
        }

        if (gsm_cmd[i].getdata == CODE_UDP_UPLINK)
        {
            if (result)
            {
                terminate_on_next=1; // exit after shut
            }
            else
            {
                result=1; // keep going on failure for code 97
            }
        }
        // delayMs (250);
        i++;
        // handle auto-retry
        if ((got_an_error)&&(doing_retry==0))
        {
#ifdef DEBUG
            usb_printf("RETRYING LAST COMMAND\r\n");
#endif
            nicedelayMs(5000);
            got_an_error = 0;
            doing_retry = 1;
            result=1; // force retry
            i--;
        }
        else
        {
            doing_retry = 0;
        }
    }
    if (result)
    {
        usb_printf ("GSM Sequence SUCCESS\r\n");
        LED_RED (0);
        LED_GREEN (0);
        LED_YELLOW (1);
        gsm_sleep_mode ();
    }
    else
    {
        usb_printf ("GSM Sequence FAILURE\r\n");
        gsm_send_command ("AT&F", "OK", 2);
        gsm_sleep_mode ();
        LED_RED (1);
        LED_GREEN (0);
    }
    return result;
}

#ifdef DEBUG
void
nicersleep (int secs)
__reentrant
{
    int i = 0;
    int j = 0;
    while (i < secs)
    {
        doServices ();
        for (j = 0; j < 10; j++)
        {
            delayMs (100);
            blink_red_led ();
        }
        i++;
    }
    LED_RED (0);
}
#endif
void
nicedelayMs (int ms)
__reentrant
{
    uint32 timeend = getMs() + ms;
    while (getMs() < timeend)
    {
        doServices ();
    }
}

void
main ()
{

    systemInit ();
    LED_YELLOW(1);		// YELLOW LED STARTS DURING POWER ON
    initUart1 ();
    P1DIR |= 0x08;		// RTS



    sleepInit ();

    lastLocation[0]='\0';

    nicedelayMs (3000);		// extra sleep for grumpy sim800 units

    LED_GREEN (1);		// Green shows when connected usb
    //usbComRequestLineStateChangeNotification(LineStateChangeCallback);

#ifdef DEBUG
    usb_connected = 1;
#else
    usb_connected = usbPowerPresent ();
#endif


    makeAllOutputsLow ();

    nicedelayMs (1000);

    loadSettingsFromFlash();

    nicedelayMs (1000);

    radioQueueInit ();
    radioQueueAllowCrcErrors = 1;
    MCSM1 = 0;


    // from xbridge3.c jstevensonsog

    MCSM0 &= 0x34;		// calibrate every fourth transition to and from IDLE.
    MCSM1 = 0x00;			// after RX go to idle, we don't transmit
    //MCSM2 = 0x08;
    MCSM2 = 0x17;			// terminate receiving on drop of carrier, but keep it up when packet quality is good.


    configBt ();

    while (1)
    {
        Dexcom_packet Pkt;
        memset (&Pkt, 0, sizeof (Dexcom_packet));
        //   boardService();

        if (get_packet (&Pkt))
        {
            print_packet (&Pkt);
        }

        RFST = 4;			// SIDLE = 4
        delayMs (100);

        radioMacSleep ();

        if (usbPowerPresent ())
        {
            sequential_missed_packets++;
        }
        if (sequential_missed_packets > 0)
        {
            int first_square =
                sequential_missed_packets * sequential_missed_packets *
                wake_earlier_for_next_miss;
            int second_square =
                (sequential_missed_packets - 1) * (sequential_missed_packets -
                                                   1) *
                wake_earlier_for_next_miss;
            int sleep_time = (268 - first_square + second_square);
            goToSleep (sleep_time);
        }
        else
        {
            goToSleep (283);
        }
        radioMacResume ();

        MCSM1 = 0;
        radioMacStrobe ();
    }
}

