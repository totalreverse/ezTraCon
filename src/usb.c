/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 *   Copyright (C) 2012 by Johann Glaser <Johann.Glaser@gmx.at>            *
 *                                                                         *
 *   Modifications for EzTraCon Copyright (C) 2019 by Michael Hipp         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/**
 * @file Defines USB descriptors, interrupt routines and helper functions.
 * To minimize code size, we make the following assumptions:
 *  - the device has exactly one configuration
 *  - and exactly one alternate setting
 *
 * Therefore, we do not have to support the Set Configuration USB request.
 */

#include "main.h"
#include "timestamp.h"


/* Define number of endpoints (except Control Endpoint 0) in a central place.
 * Be sure to include the neccessary endpoint descriptors! */
#define NUM_ENDPOINTS  2

// Analyzer always need 64 bytes EP0 paket size
#ifdef WITH_ANALYZER
#define WITH_USB_MAX_PACKET_SIZE0 (64)
#endif

// --------------------------------------------------------
// USB Device Descriptor definitions
// --------------------------------------------------------

#if   defined(WITH_EMULATE_USB_ANTFE_C)

//
// Dynastream ANTUSB2: USB 02.00, ID "3561:1941", device "01.00", ....
// Not exactly Dynastream's ANTUSB2: we show USB 01.10
//

#if !defined(WITH_USB_ENDPOINT2)
#define WITH_USB_ENDPOINT1
#endif

#ifndef WITH_USB_USB_VERSION
#define WITH_USB_USB_VERSION             (0x0110)
#endif
#ifndef WITH_USB_DEVICE_CLASS
#define WITH_USB_DEVICE_CLASS            (0)
#endif
#ifndef WITH_USB_DEVICE_SUBCLASS
#define WITH_USB_DEVICE_SUBCLASS         (0)
#endif
#ifndef WITH_USB_DEVICE_PROTOCOL
#define WITH_USB_DEVICE_PROTOCOL         (0)
#endif
#ifndef WITH_USB_MAX_PACKET_SIZE0
#define WITH_USB_MAX_PACKET_SIZE0        (32)
#endif
#ifndef WITH_USB_VENDOR_ID
#define WITH_USB_VENDOR_ID               (USB_VENDOR_ID_DYNASTREAM)
#endif
#ifndef WITH_USB_PRODUCT_ID
#define WITH_USB_PRODUCT_ID              (USB_ID_DYNASTREAM_ANTUSB2)
#endif
#ifndef WITH_USB_DEVICE_VERSION
#define WITH_USB_DEVICE_VERSION          (0x0100)
#endif
#ifndef WITH_USB_MANUFACTURER_STR_INDEX
// set to (0) or config right values for Dynastream if you have problems
#define WITH_USB_MANUFACTURER_STR_INDEX  (1)
#endif
#ifndef WITH_USB_PRODUCT_STR_INDEX
// set to (0) or config right values for Dynastream if you have problems
#define WITH_USB_PRODUCT_STR_INDEX       (2)
#endif
#ifndef WITH_USB_SERIAL_NUMBER_STR_INDEX
// set to (0) or config right values for Dynastream if you have problems
#define WITH_USB_SERIAL_NUMBER_STR_INDEX (3)
#endif
//
//

#elif defined(WITH_USB_ID_T1942_SOLID_BLUE)

//
// T1942 Solid Blue, USB 01.00, ID "3561:1941", device "00.01", ....
// Not exactly T1902 (at least in the interface part - T1902 does no enum and show all 16 IN/OUT Endpoints)
//

#ifndef WITH_USB_USB_VERSION
#define WITH_USB_USB_VERSION             (0x0100)
#endif
#ifndef WITH_USB_DEVICE_CLASS
#define WITH_USB_DEVICE_CLASS            (USB_CLASS_PER_INTERFACE)
#endif
#ifndef WITH_USB_DEVICE_SUBCLASS
#define WITH_USB_DEVICE_SUBCLASS         (USB_CLASS_PER_INTERFACE)
#endif
#ifndef WITH_USB_DEVICE_PROTOCOL
#define WITH_USB_DEVICE_PROTOCOL         (0)
#endif
#ifndef WITH_USB_MAX_PACKET_SIZE0
#define WITH_USB_MAX_PACKET_SIZE0        (64)
#endif
#ifndef WITH_USB_VENDOR_ID
#define WITH_USB_VENDOR_ID               (USB_VENDOR_ID_TACX)
#endif
#ifndef WITH_USB_PRODUCT_ID
#define WITH_USB_PRODUCT_ID              (USB_ID_T1942_SOLID_BLUE)
#endif
#ifndef WITH_USB_DEVICE_VERSION
#define WITH_USB_DEVICE_VERSION          (0x0001)
#endif
#ifndef WITH_USB_MANUFACTURER_STR_INDEX
#define WITH_USB_MANUFACTURER_STR_INDEX  (1)
#endif
#ifndef WITH_USB_PRODUCT_STR_INDEX
#define WITH_USB_PRODUCT_STR_INDEX       (2)
#endif
#ifndef WITH_USB_SERIAL_NUMBER_STR_INDEX
// set to (0) or config right values for  Tacx T1942 if you have problems
#define WITH_USB_SERIAL_NUMBER_STR_INDEX (3)
#endif
//
//

#elif defined(WITH_USB_ID_T1902_SOLID_GREEEN)
//
// T1902 Solid Green, USB 01.00, ID "3561:1902", device "20.01", ....
// Not exactly T1902 (at least in the interface part - T1902 does no enum and show all 16 IN/OUT Endpoints)
//
#ifndef WITH_USB_USB_VERSION
#define WITH_USB_USB_VERSION             (0x0100)
#endif
#ifndef WITH_USB_DEVICE_CLASS
#define WITH_USB_DEVICE_CLASS            (USB_CLASS_VENDOR_SPEC)
#endif
#ifndef WITH_USB_DEVICE_SUBCLASS
#define WITH_USB_DEVICE_SUBCLASS         (USB_CLASS_VENDOR_SPEC)
#endif
#ifndef WITH_USB_DEVICE_PROTOCOL
#define WITH_USB_DEVICE_PROTOCOL         (USB_PROTOCOL_VENDOR_SPEC)
#endif
#ifndef WITH_USB_MAX_PACKET_SIZE0
#define WITH_USB_MAX_PACKET_SIZE0        (64)
#endif
#ifndef WITH_USB_VENDOR_ID
#define WITH_USB_VENDOR_ID               (USB_VENDOR_ID_TACX)
#endif
#ifndef WITH_USB_PRODUCT_ID
#define WITH_USB_PRODUCT_ID              (USB_ID_T1902_SOLID_GREEEN)
#endif
#ifndef WITH_USB_DEVICE_VERSION
#define WITH_USB_DEVICE_VERSION          (0x2001)
#endif
#ifndef WITH_USB_MANUFACTURER_STR_INDEX
#define WITH_USB_MANUFACTURER_STR_INDEX  (0)
#endif
#ifndef WITH_USB_PRODUCT_STR_INDEX
#define WITH_USB_PRODUCT_STR_INDEX       (0)
#endif
#ifndef WITH_USB_SERIAL_NUMBER_STR_INDEX
// set to (0) or config right values for Tacx T1902 if you have problems
#define WITH_USB_SERIAL_NUMBER_STR_INDEX (3)
#endif
//
//

#endif

//
// Defaults if one or more options are not set
//

#ifndef WITH_USB_USB_VERSION
#define WITH_USB_USB_VERSION             (0x0110)
#endif
#ifndef WITH_USB_DEVICE_CLASS
#define WITH_USB_DEVICE_CLASS            (USB_CLASS_VENDOR_SPEC)
#endif
#ifndef WITH_USB_DEVICE_SUBCLASS
#define WITH_USB_DEVICE_SUBCLASS         (USB_CLASS_VENDOR_SPEC)
#endif
#ifndef WITH_USB_DEVICE_PROTOCOL
#define WITH_USB_DEVICE_PROTOCOL         (USB_PROTOCOL_VENDOR_SPEC)
#endif
#ifndef WITH_USB_MAX_PACKET_SIZE0
#define WITH_USB_MAX_PACKET_SIZE0        (64)
#endif
#ifndef WITH_USB_VENDOR_ID
#define WITH_USB_VENDOR_ID               (USB_VENDOR_ID_TACX)
#endif
#ifndef WITH_USB_PRODUCT_ID
#define WITH_USB_PRODUCT_ID              (USB_ID_DEVELOP)
#endif
#ifndef WITH_USB_DEVICE_VERSION
#define WITH_USB_DEVICE_VERSION          (timestampC)
#endif
#ifndef WITH_USB_MANUFACTURER_STR_INDEX
#define WITH_USB_MANUFACTURER_STR_INDEX  (1)
#endif
#ifndef WITH_USB_PRODUCT_STR_INDEX
#define WITH_USB_PRODUCT_STR_INDEX       (2)
#endif
#ifndef WITH_USB_SERIAL_NUMBER_STR_INDEX
#define WITH_USB_SERIAL_NUMBER_STR_INDEX (3)
#endif

#if !defined(WITH_USB_ENDPOINT2) && !defined(WITH_USB_ENDPOINT1)
#define WITH_USB_ENDPOINT2
#endif


/*
 * Normally, we would initialize the descriptor structures in C99 style:
 *
 * __code usb_device_descriptor_t device_descriptor = {
 *   .bLength = foo,
 *   .bDescriptorType = bar,
 *   .bcdUSB = 0xABCD,
 *   ...
 * };
 *
 * But SDCC currently does not support this, so we have to do it the
 * old-fashioned way...
 */

// Note:
/* WARNING: ALL config, interface and endpoint descriptors MUST be adjacent! */

#ifdef WITH_LOAD_DATA_TO_UPPER_MEM
__code __at 0x1b80  struct usb_all_descriptors all_descriptors = {
#else
__code struct usb_all_descriptors all_descriptors = {
#endif
 {
  /* .bLength = */             sizeof(struct usb_device_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_DEVICE,
  /* .bcdUSB = */              WITH_USB_USB_VERSION,
  /* .bDeviceClass = */        WITH_USB_DEVICE_CLASS,
  /* .bDeviceSubClass = */     WITH_USB_DEVICE_SUBCLASS,
  /* .bDeviceProtocol = */     WITH_USB_DEVICE_PROTOCOL,
  /* .bMaxPacketSize0 = */     WITH_USB_MAX_PACKET_SIZE0,
  /* .idVendor = */            WITH_USB_VENDOR_ID,
  /* .idProduct = */           WITH_USB_PRODUCT_ID,
  /* .bcdDevice = */           WITH_USB_DEVICE_VERSION,
  /* .iManufacturer = */       WITH_USB_MANUFACTURER_STR_INDEX,
  /* .iProduct = */            WITH_USB_PRODUCT_STR_INDEX,
  /* .iSerialNumber = */       WITH_USB_SERIAL_NUMBER_STR_INDEX,
  /* .bNumConfigurations = */  1
},


#ifdef WITH_USB_ENDPOINT1
 {
  /* .bLength = */             sizeof(struct usb_config_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_CONFIGURATION,
  /* .wTotalLength = */        sizeof(struct usb_config_descriptor) +
                               sizeof(struct usb_interface_descriptor) +
                               (NUM_ENDPOINTS *
                               sizeof(struct usb_endpoint_descriptor)),
  /* .bNumInterfaces = */      1,
  /* .bConfigurationValue = */ 1,
  /* .iConfiguration = */      0,                           /* String-idx describing this configuration */
  /* .bmAttributes = */        USB_CONFIG_ATTRIB_RESERVED,  /* Fortius |= USB_CONFIG_ATTRIB_REMOTE_WAKEUP */
  /* .MaxPower = */            0x80                         /* imagic = 50*2 = 100 mA */
},

 {
  /* .bLength = */             sizeof(struct usb_interface_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_INTERFACE,
  /* .bInterfaceNumber = */    0,
  /* .bAlternateSetting = */   0,
  /* .bNumEndpoints = */       NUM_ENDPOINTS,
  /* .bInterfaceClass = */     USB_CLASS_VENDOR_SPEC,
  /* .bInterfaceSubclass = */  0, //USB_CLASS_VENDOR_SPEC,
  /* .bInterfaceProtocol = */  0, // USB_PROTOCOL_VENDOR_SPEC,
  /* .iInterface = */          0 // 5  /* String-idx describing this interface */
},

 {
  /* .bLength = */             sizeof(struct usb_endpoint_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_ENDPOINT,
  /* .bEndpointAddress = */    1 | USB_DIR_IN,
  /* .bmAttributes = */        USB_ENDPOINT_TYPE_BULK,
  /* .wMaxPacketSize = */      64,
  /* .bInterval = */           0
},

{
  /* .bLength = */             sizeof(struct usb_endpoint_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_ENDPOINT,
  /* .bEndpointAddress = */    1 | USB_DIR_OUT,
  /* .bmAttributes = */        USB_ENDPOINT_TYPE_BULK,
  /* .wMaxPacketSize = */      64,
  /* .bInterval = */           0
},

#else
// Not exactly T1902 (at least in the interface part - 1902 does no enum and show all 16 IN/OUT Endpoints)
{
  /* .bLength = */             sizeof(struct usb_config_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_CONFIGURATION,
  /* .wTotalLength = */        sizeof(struct usb_config_descriptor) +
                               sizeof(struct usb_interface_descriptor) +
                               (NUM_ENDPOINTS *
                               sizeof(struct usb_endpoint_descriptor)),
  /* .bNumInterfaces = */      1,
  /* .bConfigurationValue = */ 1,
  /* .iConfiguration = */      0,                           /* String-idx describing this configuration */
  /* .bmAttributes = */        USB_CONFIG_ATTRIB_RESERVED,  /* Fortius |= USB_CONFIG_ATTRIB_REMOTE_WAKEUP */
  /* .MaxPower = */            0                            /* imagic = 50*2 = 100 mA */
},

{
  /* .bLength = */             sizeof(struct usb_interface_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_INTERFACE,
  /* .bInterfaceNumber = */    0,
  /* .bAlternateSetting = */   0,
  /* .bNumEndpoints = */       NUM_ENDPOINTS,
  /* .bInterfaceClass = */     USB_CLASS_VENDOR_SPEC,
  /* .bInterfaceSubclass = */  0, //USB_CLASS_VENDOR_SPEC,
  /* .bInterfaceProtocol = */  0, // USB_PROTOCOL_VENDOR_SPEC,
  /* .iInterface = */          0 // 5  /* String-idx describing this interface */
},

{
  /* .bLength = */             sizeof(struct usb_endpoint_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_ENDPOINT,
  /* .bEndpointAddress = */    2 | USB_DIR_IN,
  /* .bmAttributes = */        USB_ENDPOINT_TYPE_BULK,
  /* .wMaxPacketSize = */      64,
  /* .bInterval = */           0
},

{
  /* .bLength = */             sizeof(struct usb_endpoint_descriptor),
  /* .bDescriptorType = */     USB_DESCRIPTOR_TYPE_ENDPOINT,
  /* .bEndpointAddress = */    2 | USB_DIR_OUT,
  /* .bmAttributes = */        USB_ENDPOINT_TYPE_BULK,
  /* .wMaxPacketSize = */      64,
  /* .bInterval = */           0
}
#endif
};

#define STR_DESCR_0(len) ((USB_DESCRIPTOR_TYPE_STRING<<8)+((len)*2+2))

// STR_DESCR_0(7)  , 'T','a','c','x',' ','b','v'    
// STR_DESCR_0(12) , 'T','a','c','x',' ','F','o','r','t','i','u','s'
// STR_DESCR_0(0) 
__code uint16_t buffer[] = {
    STR_DESCR_0(1), USB_LANG_ENGLISH_US,
    STR_DESCR_0(3), 'M','H','I',
    STR_DESCR_0(8), 'E','z','T','r','a','C','o','n',
    timestampStr
};
__code uint8_t str_descr_offsets[4] = { 0, 2, 2+4, 2+4+9 }; 


/* Also update external declarations in "include/usb.h" if making changes to
 * these variables! */
#ifndef WITH_USB_POLL
__bit volatile Semaphore_USB_Setup = false;
__bit volatile Semaphore_EP12_out = false;
#endif

/* Convenience functions */
#define STALL_EP0()   EP0CS |= EP0STALL
#define CLEAR_IRQ()   EXIF &= ~USBINT


volatile __xdata __at 0x7FE8 struct setup_data setup_data;

// change offset if STR_DESCRIPTORS are changed

#ifndef WITH_USB_POLL
void sudav_isr(void) __interrupt SUDAV_ISR {
  Semaphore_USB_Setup = true;
  STATISTICS_IRQ(count_sudav_isr++);

  CLEAR_IRQ();
  /* Do not clear an IRQ bit by reading an IRQ register, ORing its contents with a bit mask, and writ-
     ing back the IRQ register. This will clear ALL pending interrupts. Instead, simply write the bit
     mask value (with the IRQ you want to clear) directly to the IRQ register. */
  USBIRQ = SUDAVIR;
}
/**
 * EP1 or EP2 IN: called after the transfer from uC->Host has finished: we sent data
 */
void ep12in_isr(void)    __interrupt EP2IN_ISR {
  STATISTICS_IRQ(count_ep12_in++);
#if 0
  Semaphore_EP12_in = true;
  CLEAR_IRQ();
  /* Do not clear an IRQ bit by reading an IRQ register, ORing its contents with a bit mask, and writ-
     ing back the IRQ register. This will clear ALL pending interrupts. Instead, simply write the bit
     mask value (with the IRQ you want to clear) directly to the IRQ register. */
  IN07IRQ = IN1IR|IN2IR;     // Clear OUT1,OUT2 IRQ
#endif
}

/**
 * EP1 or EP2 OUT: called after the transfer from Host->uC has finished: we got data
 */
void ep12out_isr(void)   __interrupt EP2OUT_ISR {
  STATISTICS_IRQ(count_ep12_out++);

  Semaphore_EP12_out = 1;

  CLEAR_IRQ();
  /* Do not clear an IRQ bit by reading an IRQ register, ORing its contents with a bit mask, and writ-
     ing back the IRQ register. This will clear ALL pending interrupts. Instead, simply write the bit
     mask value (with the IRQ you want to clear) directly to the IRQ register. */
  OUT07IRQ = OUT1IR|OUT2IR;    // Clear OUT1|OUT2 IRQ
}
#endif

// start of frame could be used as a "timer" as long as the host sends the SOF packets (every ms)
#if 0
void sof_isr(void)      __interrupt SOF_ISR      {
    CLEAR_IRQ();
    USBIRQ = SOFIR;
}

void sutok_isr(void)    __interrupt SUTOK_ISR    {
    CLEAR_IRQ();
    USBIRQ = SUTOKIR;
}

void suspend_isr(void)  __interrupt SUSPEND_ISR  {
    CLEAR_IRQ();
    USBIRQ = SUSPIR;
}

void usbreset_isr(void) __interrupt USBRESET_ISR {
    CLEAR_IRQ();
    USBIRQ = URESIR;
}

void ibn_isr(void)      __interrupt IBN_ISR      { }
void ep0in_isr(void)    __interrupt EP0IN_ISR    { }
void ep0out_isr(void)   __interrupt EP0OUT_ISR   { }
#endif


/**
 * Return the control/status register for an endpoint
 *
 * @param ep endpoint address
 * @return on success: pointer to Control & Status register for endpoint
 *  specified in \a ep
 * @return on failure: NULL
 */
#ifdef WITH_ASM8051
// ASM to squeeze some code bytes
static __xdata uint8_t* usb_get_endpoint_cs_reg() __naked  {
    __asm
    ; EP0CS,         0x7FB4
    ; setup_data     0x7FE8  
    mov     dptr,#(_setup_data + 0x0004)   

    movx    a,@dptr
    rlc     a
    anl     a,#0x1e                       ;  0x1e = (USB_ENDPOINT_ADDRESS_MASK<<1)
    jz      usb_get_endpoint_cs_reg_2
    swap    a
    cpl     c
    rrc     a
    jnc     usb_get_endpoint_cs_reg_1
    mov     dptr,#0x0000
    ret
usb_get_endpoint_cs_reg_1:        
    swap    a
    rl      a
usb_get_endpoint_cs_reg_2:
    add     a,#0xb4                       ; EP0CS = 0x7FB4
    mov     dpl,a
    ret

    __endasm;
}

#else
static __xdata uint8_t* usb_get_endpoint_cs_reg() {
  /* Mask direction bit */
  uint8_t ep = setup_data.wIndex;
  uint8_t ep_num = ep & USB_ENDPOINT_ADDRESS_MASK;

  if(!ep_num)
      return &EP0CS;
  if(ep_num & ~0x07)
      return (void*)0;
  // not portable ..
  return ep & USB_ENDPOINT_DIR_MASK ? &(&EP0CS)[ep_num<<1] : &(&EP0CS+0x10)[ep_num<<1];
}
#endif

inline void usb_reset_data_toggle(uint8_t ep) {
  /* TOGCTL register:
     +----+-----+-----+------+-----+-------+-------+-------+
     | Q  |  S  |  R  |  IO  |  0  |  EP2  |  EP1  |  EP0  |
     +----+-----+-----+------+-----+-------+-------+-------+

     To reset data toggle bits, we have to write the endpoint direction (IN/OUT)
     to the IO bit and the endpoint number to the EP2..EP0 bits. Then, in a
     separate write cycle, the R bit needs to be set.
  */
  uint8_t togctl_value = (ep & USB_ENDPOINT_DIR_MASK >> 3) | (ep & 0x7);

  /* First step: Write EP number and direction bit */
  TOGCTL = togctl_value;

  /* Second step: Set R bit */
  togctl_value |= TOG_R;
  TOGCTL = togctl_value;
}

/**
 * Handle GET_STATUS request.
 *
 * @return on success: true
 * @return on failure: false
 */
bool usb_handle_get_status(void) {
  uint8_t *ep_cs;

  IN0BUF[0] = 0;
  IN0BUF[1] = 0;
  switch (setup_data.bmRequestType) {
  case (uint8_t)USB_RECIP_GS_ENDPOINT:
    /* Get stall bit for endpoint specified in low byte of wIndex */
    ep_cs = usb_get_endpoint_cs_reg();
    if(!ep_cs)
      goto usb_handle_get_status_false;

    if (*ep_cs & EPSTALL) {
      IN0BUF[0] = 0x01;
    }
    /* Second byte sent has to be always zero */
    // fall-through
  case (uint8_t)USB_RECIP_GS_DEVICE:
    /* Two byte response: Byte 0, Bit 0 = self-powered, Bit 1 = remote wakeup.
     *                    Byte 1: reserved, reset to zero */
    // fall-through
  case (uint8_t)USB_RECIP_GS_INTERFACE:
    /* Always return two zero bytes according to USB 1.1 spec, p. 191 */

    /* Send response */
    IN0BC = 2;
    return true;
  }
usb_handle_get_status_false:
  return false;
}

/**
 * Handle CLEAR_FEATURE request.
 *
 * @return on success: true
 * @return on failure: false
 */
bool usb_handle_clear_feature(void) {
  __xdata uint8_t *ep_cs;

  switch (setup_data.bmRequestType) {
  case (uint8_t)USB_RECIP_CF_DEVICE:
    /* Clear remote wakeup not supported: stall EP0 */
    goto usb_handle_clear_feature_stall;

  case (uint8_t)USB_RECIP_CF_ENDPOINT:
    if (setup_data.wValue == 0) {
      /* Unstall the endpoint specified in wIndex */
      ep_cs = usb_get_endpoint_cs_reg();
      if (!ep_cs) {
        return false;
      }
      *ep_cs &= ~EPSTALL;
    }
    else {
      /* Unsupported feature, stall EP0 */
usb_handle_clear_feature_stall:
      STALL_EP0();
    }
    break;
  default:
    /* Vendor commands... */
  }

  return true;
}

/**
 * Handle SET_FEATURE request.
 *
 * @return on success: true
 * @return on failure: false
 */
static bool usb_handle_set_feature(void) {
  __xdata uint8_t *ep_cs;

  switch (setup_data.bmRequestType) {
  case (uint8_t)USB_RECIP_SF_DEVICE:
    // if (setup_data.wValue == 2) {
    //   return true;
    // }
    break;
  case (uint8_t)USB_RECIP_SF_ENDPOINT:
    if (setup_data.wValue == 0) {
      /* Stall the endpoint specified in wIndex */
      ep_cs = usb_get_endpoint_cs_reg();
      if (!ep_cs) {
        goto usb_handle_set_feature_false;
      }
      *ep_cs |= EPSTALL;
    }
    else {
      /* Unsupported endpoint feature */
usb_handle_set_feature_false:
      return false;
    }
    break;
  default:
    /* Vendor commands... */
    break;
  }

  return true;
}

/**
 * Handle GET_DESCRIPTOR request.
 *
 * @return on success: true
 * @return on failure: false
 */
static bool usb_handle_get_descriptor(void) {
  uint8_t offset = 0;
  switch (HI8(setup_data.wValue)) {
  case (uint8_t)USB_DESCRIPTOR_TYPE_DEVICE:
    SUDPTRH = HI8(&all_descriptors.device_descriptor);
    SUDPTRL = LO8(&all_descriptors.device_descriptor);
    break;
  case (uint8_t)USB_DESCRIPTOR_TYPE_CONFIGURATION:
    SUDPTRH = HI8(&all_descriptors.config_descriptor);
    SUDPTRL = LO8(&all_descriptors.config_descriptor);
    break;
  case (uint8_t)USB_DESCRIPTOR_TYPE_STRING:
    if (setup_data.wIndex == USB_LANG_ENGLISH_US) {
      uint8_t descriptor_index = LO8(setup_data.wValue);
      if(descriptor_index >= sizeof(str_descr_offsets)) {
        goto usb_handle_get_descriptor_false;
      }
      offset = str_descr_offsets[descriptor_index];
      /* Supply string descriptor */
    } else if (setup_data.wIndex != 0) {
      goto usb_handle_get_descriptor_false;
    }
    /* default offset = 0 => Supply language descriptor */
    SUDPTRH = HI8(&buffer[offset]);
    SUDPTRL = LO8(&buffer[offset]);
    break;
  default:
    /* Unsupported descriptor type */
usb_handle_get_descriptor_false:    
    return false;
    break;
  }

  return true;
}

/**
 * Handle SET_INTERFACE request.
 */
static void usb_handle_set_interface(void) {
  /* Reset Data Toggle */

#ifdef WITH_EMULATE_USB_ANTFE_C

  usb_reset_data_toggle(USB_DIR_IN  | 1);
  usb_reset_data_toggle(USB_DIR_OUT | 1);
  /* Unstall & clear busy flag of all valid IN endpoints */
  IN1CS = 0 | EPBSY;

  /* Unstall all valid OUT endpoints, reset bytecounts */
  OUT1CS = 0;
  OUT1BC = 0;

#else

  usb_reset_data_toggle(USB_DIR_IN  | 2);
  usb_reset_data_toggle(USB_DIR_OUT | 2);

  /* Unstall & clear busy flag of all valid IN endpoints */
  IN2CS = 0 | EPBSY;

  /* Unstall all valid OUT endpoints, reset bytecounts */
  OUT2CS = 0;
  OUT2BC = 0;

#endif
}

/**
 * Handle the arrival of a USB Control Setup Packet.
 */
void usb_handle_setup_data(void) {

#ifdef WITH_USB_POLL
    if(OUT07IRQ & OUT0IR) {
        STATISTICS_IRQ(count_sudav_isr++);
        CLEAR_IRQ();
        /* Do not clear an IRQ bit by reading an IRQ register, ORing its contents with a bit mask, and writ-
            ing back the IRQ register. This will clear ALL pending interrupts. Instead, simply write the bit
            mask value (with the IRQ you want to clear) directly to the IRQ register. */
        OUT07IRQ = OUT0IR;
#ifdef WITH_ANALYZER
        handle_analyzer(true);
#endif
        return;
    } else if(USBIRQ & SUDAVIR) {
        STATISTICS_IRQ(count_sudav_isr++);
        CLEAR_IRQ();
        /* Do not clear an IRQ bit by reading an IRQ register, ORing its contents with a bit mask, and writ-
            ing back the IRQ register. This will clear ALL pending interrupts. Instead, simply write the bit
            mask value (with the IRQ you want to clear) directly to the IRQ register. */
        USBIRQ = SUDAVIR;
#else
    if(Semaphore_USB_Setup) {
       Semaphore_USB_Setup = false;
#endif

    switch (setup_data.bRequest) {
    case (uint8_t)USB_REQ_GET_STATUS:
      if (!usb_handle_get_status()) {
        goto usb_stall_ep0;
      }
      break;
    case (uint8_t)USB_REQ_CLEAR_FEATURE:
      if (!usb_handle_clear_feature()) {
        goto usb_stall_ep0;
      }
      break;
    case (uint8_t)USB_REQ_SET_FEATURE:
      if (!usb_handle_set_feature()) {
        goto usb_stall_ep0;
      }
      break;
    case (uint8_t)USB_REQ_GET_DESCRIPTOR:
      if (!usb_handle_get_descriptor()) {
        goto usb_stall_ep0;
      }
      break;

    case (uint8_t)USB_REQ_GET_CONFIGURATION:
      /* we have only one configuration, return its index */
      IN0BUF[0] = all_descriptors.config_descriptor.bConfigurationValue;
      goto usb_in0bc_to_1;
    case (uint8_t)USB_REQ_GET_INTERFACE:
      /* we have only one interface, return its number */
      IN0BUF[0] = all_descriptors.interface_descriptor00.bInterfaceNumber;
usb_in0bc_to_1:
      IN0BC = 1;
      break;

    case (uint8_t)USB_REQ_SET_INTERFACE:
      usb_handle_set_interface();
      break;

    case (uint8_t)USB_REQ_SET_ADDRESS:
      /* Handled by USB core */
    case (uint8_t)USB_REQ_SET_CONFIGURATION:
      /* we have only one configuration -> nothing to do */
    case (uint8_t)USB_REQ_SYNCH_FRAME:
      /* Isochronous endpoints not used -> nothing to do */
      break;

    case (uint8_t)USB_REQ_SET_DESCRIPTOR:  /* Set Descriptor not supported. */      
    case 2:                                /* Reserved value */    
    case 4:                                /* Reserved value */
usb_stall_ep0:
      STALL_EP0();
      break;

    default:
#ifdef WITH_ANALYZER
      handle_analyzer(false);
#endif
      break;
  }

  EP0CS |= HSNAK;
  }
}

/**
 * USB initialization. Configures USB interrupts, endpoints and performs
 * ReNumeration.
 */
void usb_init(void) {

#ifdef WITH_EMULATE_USB_ANTFE_C
  /* Mark endpoint 1 IN & OUT as valid */
  IN07VAL  = IN1VAL;
  OUT07VAL = OUT1VAL;
#else
  /* Mark endpoint 2 IN & OUT as valid */
  IN07VAL  = IN2VAL;
  OUT07VAL = OUT2VAL;
  // IN2BUF double-buffering no longer used - increased complexity just to save one buffer copy
  // USBPAIR  = PR2IN;
#endif

  /* Make sure no isochronous endpoints are marked valid */
  INISOVAL  = 0;
  OUTISOVAL = 0;

  /* Disable isochronous endpoints. This makes the isochronous data buffers
   * available as 8051 XDATA memory at address 0x2000 - 0x27FF */
  ISOCTL = ISODISAB;

#ifndef WITH_USB_POLL
  /* Enable USB Autovectoring */
  USBBAV |= AVEN;

  /* Enable SUDAV interrupt */
    // Enable URESIE|SUSPIE|SUTOKIE|SUDAV interrup
    // b0: _SUDAV Setupdata Availble
    // b2: _SUTOKIE SETUP Token Interrupt Enable
    // b3: _SUSPIE USB Suspend Interrupt Enable
    // b4: _URESIE USB Reset Interrupt Enable
  USBIEN |= SUDAVIE;
  //USBIEN |= URESIE; // do "re-arm" the IN2buf for double buffering
  //USBIEN |= SOFIE;  // as alternate timer (1ms) ?
  //USBIEN |= SUSPIE;  // as alternate timer (1ms) ?
  //USBIEN |= SUTOKIR; // only for debug?

#ifdef WITH_EMULATE_USB_ANTFE_C
  /* Enable EP2 OUT (no IN) interrupts */
  OUT07IEN = OUT1IEN;
  //IN07IEN  = IN2IEN;
#else
  /* Enable EP2 OUT (no IN) interrupts */
  OUT07IEN = OUT2IEN;
  //IN07IEN  = IN2IEN;
#endif

  PUSB = false;
  /* Enable USB interrupt (EIE register) */
  EUSB = true;
#endif

}

// AN21xx TRM says: "To simulate a USB
// disconnect, the 8051 writes the value 00001010 to USBCS. This floats the DISCON# pin, and
// provides an internal DISCON signal to the USB core that causes it to perform disconnect
// housekeeping. To re-connect to USB, the 8051 writes the value 00000110 to USBCS. This presents
// a logic HI to the DISCON# pin, enables the output buffer, and sets the RENUM bit HI to
// indicate that the 8051 (and not the USB core) is now in control for USB transfers"

void usb_renumber() {
  uint16_t i;
  /* Perform ReNumeration */
  USBCS = DISCON | RENUM;

  for (i = 0; i < 60000; i++); //   delay_ms(200);

  // ack. 'historic' IRQs
  IN07IRQ  = 0xff;
  OUT07IRQ = 0xff;
  USBIRQ   = 0xff;
  EXIF    &= ~USBINT;

  USBCS = DISCOE | RENUM;
}

