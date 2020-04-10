/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 *   Copyright (C) 2012 by Johann Glaser <Johann.Glaser@gmx.at>            *
 *                                                                         *
 *   Modifications for EzTraCOn Copyright (C) 2019 by Michael Hipp         *
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

#ifndef __USB_H
#define __USB_H


#include <stdint.h>
#include <stdbool.h>

/* High and Low byte of a word (uint16_t) */
#define HI8(word)   (uint8_t)(((uint16_t)word >> 8) & 0xff)
#define LO8(word)   (uint8_t)((uint16_t)word & 0xff)

// USB Vendor IDs
#define USB_VENDOR_ID_TACX                  (0x3561)
#define USB_VENDOR_ID_DYNASTREAM            (0x0fcf)

// USB Device IDs
#define USB_ID_DEVELOP                      (0x1900)
#define USB_ID_T1942_SOLID_BLUE_NOINIT      (0xE6BE)
#define USB_ID_T1902_SOLID_GREEEN           (0x1902)
#define USB_ID_T1942_SOLID_BLUE             (0x1942)
// for emulating one of the non ez-usb devices:
#define USB_ID_T1904_WHITE_GREEN            (0x1904)
#define USB_ID_T1932_WHITE_BLUE             (0x1932)

// Dynastream USB devices
#define USB_ID_DYNASTREAM_ANTUSB            (0x1004)
#define USB_ID_DYNASTREAM_ANTUSB2           (0x1008)
#define USB_ID_DYNASTREAM_ANTUSB_M          (0x1009)


/*********** USB descriptors. See section 9.5 of the USB 1.1 spec **********/

/* USB Descriptor Types. See USB 1.1 spec, page 187, table 9-5 */
#define USB_DESCRIPTOR_TYPE_DEVICE         0x01
#define USB_DESCRIPTOR_TYPE_CONFIGURATION  0x02
#define USB_DESCRIPTOR_TYPE_STRING         0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE      0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT       0x05

#define USB_DESCRIPTOR_TYPE_HID            0x21
#define USB_DESCRIPTOR_TYPE_REPORT         0x22
#define USB_DESCRIPTOR_TYPE_PHYSICAL       0x23
#define USB_DESCRIPTOR_TYPE_HUB            0x29

/* USB Device and/or Interface Classes */
#define USB_CLASS_PER_INTERFACE         0       /* only used for bDeviceClass */
#define USB_CLASS_AUDIO                 1
#define USB_CLASS_COMM                  2
#define USB_CLASS_HID                   3
#define USB_CLASS_PRINTER               7
#define USB_CLASS_PTP                   6
#define USB_CLASS_MASS_STORAGE          8
#define USB_CLASS_HUB                   9
#define USB_CLASS_DATA                  10
#define USB_CLASS_VENDOR_SPEC           0xff

/* use for usb_device_descriptor.bDeviceProtocol, USB 1.1 spec, page 198, table 9-7 */
#define USB_PROTOCOL_VENDOR_SPEC        0xff

/* use for usb_config_descriptor.bmAttributes, USB 1.1 spec, page 200, table 9-8 */
#define USB_CONFIG_ATTRIB_RESERVED      0x80
#define USB_CONFIG_ATTRIB_SELF_POWERED  0x40
#define USB_CONFIG_ATTRIB_REMOTE_WAKEUP 0x20

/* use for usb_endpoint_descriptor.bEndpointAddress, USB 1.1 spec, page 203, table 9-10 */
#define USB_ENDPOINT_ADDRESS_MASK       0x0f
#define USB_ENDPOINT_DIR_MASK           0x80

/* use for usb_endpoint_descriptor.bmAttributes, USB 1.1 spec, page 204, table 9-10 */
#define USB_ENDPOINT_TYPE_MASK          0x03
#define USB_ENDPOINT_TYPE_CONTROL       0
#define USB_ENDPOINT_TYPE_ISOCHRONOUS   1
#define USB_ENDPOINT_TYPE_BULK          2
#define USB_ENDPOINT_TYPE_INTERRUPT     3

#define USB_LANG_ENGLISH_US   (0x009 | (0x01 << 10))
#define USB_LANG_ENGLISH_UK   (0x009 | (0x02 << 10))
#define USB_LANG_GERMAN       (0x007 | (0x01 << 10))
#define USB_LANG_GERMAN_CH    (0x007 | (0x02 << 10))
#define USB_LANG_GERMAN_AT    (0x007 | (0x03 << 10))
#define USB_LANG_GERMAN_LU    (0x007 | (0x04 << 10))
#define USB_LANG_GERMAN_LI    (0x007 | (0x05 << 10))


#ifndef EZTRACON_8051
#pragma pack(push,1)
#endif


/** USB Device Descriptor. See USB 1.1 spec, pp. 196 - 198 */
struct usb_device_descriptor {
  uint8_t  bLength;            ///< Size of this descriptor in bytes.
  uint8_t  bDescriptorType;    ///< DEVICE Descriptor Type.
  uint16_t bcdUSB;             ///< USB specification release number (BCD).
  uint8_t  bDeviceClass;       ///< Class code.
  uint8_t  bDeviceSubClass;    ///< Subclass code.
  uint8_t  bDeviceProtocol;    ///< Protocol code.
  uint8_t  bMaxPacketSize0;    ///< Maximum packet size for EP0 (8, 16, 32, 64).
  uint16_t idVendor;           ///< USB Vendor ID.
  uint16_t idProduct;          ///< USB Product ID.
  uint16_t bcdDevice;          ///< Device Release Number (BCD).
  uint8_t  iManufacturer;      ///< Index of manufacturer string descriptor.
  uint8_t  iProduct;           ///< Index of product string descriptor.
  uint8_t  iSerialNumber;      ///< Index of string descriptor containing serial #.
  uint8_t  bNumConfigurations; ///< Number of possible configurations.
};

/** USB Configuration Descriptor. See USB 1.1 spec, pp. 199 - 200 */
struct usb_config_descriptor {
  uint8_t  bLength;            ///< Size of this descriptor in bytes.
  uint8_t  bDescriptorType;    ///< CONFIGURATION descriptor type.
  uint16_t wTotalLength;       ///< Combined total length of all descriptors.
  uint8_t  bNumInterfaces;     ///< Number of interfaces in this configuration.
  uint8_t  bConfigurationValue;///< Value used to select this configuration.
  uint8_t  iConfiguration;     ///< Index of configuration string descriptor.
  uint8_t  bmAttributes;       ///< Configuration characteristics.
  uint8_t  MaxPower;           ///< Maximum power consumption in 2 mA units.
};

/** USB Interface Descriptor. See USB 1.1 spec, pp. 201 - 203 */
struct usb_interface_descriptor {
  uint8_t  bLength;            ///< Size of this descriptor in bytes.
  uint8_t  bDescriptorType;    ///< INTERFACE descriptor type.
  uint8_t  bInterfaceNumber;   ///< Interface number.
  uint8_t  bAlternateSetting;  ///< Value used to select alternate setting.
  uint8_t  bNumEndpoints;      ///< Number of endpoints used by this interface.
  uint8_t  bInterfaceClass;    ///< Class code.
  uint8_t  bInterfaceSubclass; ///< Subclass code.
  uint8_t  bInterfaceProtocol; ///< Protocol code.
  uint8_t  iInterface;         ///< Index of interface string descriptor.
};

/** USB Endpoint Descriptor. See USB 1.1 spec, pp. 203 - 204 */
struct usb_endpoint_descriptor {
  uint8_t  bLength;            ///< Size of this descriptor in bytes.
  uint8_t  bDescriptorType;    ///< ENDPOINT descriptor type.
  uint8_t  bEndpointAddress;   ///< Endpoint Address: USB 1.1 spec, table 9-10.
  uint8_t  bmAttributes;       ///< Endpoint Attributes: USB 1.1 spec, table 9-10.
  uint16_t wMaxPacketSize;     ///< Maximum packet size for this endpoint.
  uint8_t  bInterval;          ///< Polling interval (in ms) for this endpoint.
};

/** USB Language Descriptor. See USB 1.1 spec, pp. 204 - 205 */
struct usb_language_descriptor {
  uint8_t  bLength;            ///< Size of this descriptor in bytes.
  uint8_t  bDescriptorType;    ///< STRING descriptor type.
  uint16_t wLANGID[];          ///< LANGID codes.
};

/** USB String Descriptor. See USB 1.1 spec, pp. 204 - 205 */
struct usb_string_descriptor {
  uint8_t  bLength;            ///< Size of this descriptor in bytes.
  uint8_t  bDescriptorType;    ///< STRING descriptor type.
  uint16_t bString[];          ///< UNICODE encoded string.
};

/********************** USB Control Endpoint 0 related *********************/

/** USB Control Setup Data. See USB 1.1 spec, pp. 183 - 185 */
struct setup_data {
  uint8_t  bmRequestType;      ///< Characteristics of a request.
  uint8_t  bRequest;           ///< Specific request.
  uint16_t wValue;             ///< Field that varies according to request.
  uint16_t wIndex;             ///< Field that varies according to request.
  uint16_t wLength;            ///< Number of bytes to transfer in data stage.
};

/************************** application specific stuff **************************/

struct usb_all_descriptors {
   struct usb_device_descriptor device_descriptor;
   struct usb_config_descriptor config_descriptor;
   struct usb_interface_descriptor interface_descriptor00;
   struct usb_endpoint_descriptor Bulk_EPx_IN_Endpoint_Descriptor;
   struct usb_endpoint_descriptor Bulk_EPx_OUT_Endpoint_Descriptor;   
};

#ifndef EZTRACON_8051
#pragma pack(pop)
#endif

/*
 * USB Request Types (bmRequestType): See USB 1.1 spec, page 183, table 9-2
 *
 * Bit 7: Data transfer direction
 *    0 = Host-to-device
 *    1 = Device-to-host
 * Bit 6...5: Type
 *    0 = Standard
 *    1 = Class
 *    2 = Vendor
 *    3 = Reserved
 * Bit 4...0: Recipient
 *    0 = Device
 *    1 = Interface
 *    2 = Endpoint
 *    3 = Other
 *    4...31 = Reserved
 */

#define USB_DIR_OUT             0x00
#define USB_DIR_IN              0x80

#define USB_REQ_TYPE_STANDARD   (0x00 << 5)
#define USB_REQ_TYPE_CLASS      (0x01 << 5)
#define USB_REQ_TYPE_VENDOR     (0x02 << 5)
#define USB_REQ_TYPE_RESERVED   (0x03 << 5)

#define USB_RECIP_DEVICE        0x00
#define USB_RECIP_INTERFACE     0x01
#define USB_RECIP_ENDPOINT      0x02
#define USB_RECIP_OTHER         0x03

/* bmRequestType for USB Standard Requests */

/* Clear Interface Request */
#define USB_RECIP_CF_DEVICE    (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define USB_RECIP_CF_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define USB_RECIP_CF_ENDPOINT  (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Get Configuration Request */
#define USB_RECIP_GC_DEVICE    (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Get Descriptor Request */
#define USB_RECIP_GD_DEVICE    (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Get Interface Request */
#define USB_RECIP_GI_INTERFACE (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)

/* Get Status Request: See USB 1.1 spec, page 190 */
#define USB_RECIP_GS_DEVICE    (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define USB_RECIP_GS_INTERFACE (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define USB_RECIP_GS_ENDPOINT  (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Set Address Request is handled by EZ-USB core */

/* Set Configuration Request */
#define USB_RECIP_SC_DEVICE    (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Set Descriptor Request */
#define USB_RECIP_SD_DEVICE    (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)

/* Set Feature Request */
#define USB_RECIP_SF_DEVICE    (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_DEVICE)
#define USB_RECIP_SF_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define USB_RECIP_SF_ENDPOINT  (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* Set Interface Request */
#define USB_RECIP_SI_INTERFACE (USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_RECIP_INTERFACE)

/* Synch Frame Request */
#define USB_RECIP_SY_ENDPOINT  (USB_DIR_IN | USB_REQ_TYPE_STANDARD | USB_RECIP_ENDPOINT)

/* USB Requests (bRequest): See USB 1.1 spec, table 9-4 on page 187 */
#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
// Value '2' is reserved for future use
#define USB_REQ_SET_FEATURE             0x03
// Value '4' is reserved for future use
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C

/* Standard Feature Selectors: See USB 1.1 spec, table 9-6 on page 188 */
#define DEVICE_REMOTE_WAKEUP     1
#define ENDPOINT_HALT            0


/* External declarations for variables that need to be accessed outside of the USB module */
#ifndef WITH_USB_POLL
extern __bit volatile Semaphore_EP12_out;
extern __bit volatile Semaphore_USB_Setup;
#endif
extern volatile __xdata __at 0x7FE8 struct setup_data setup_data;



/*************************** Function Prototypes ***************************/

extern inline void usb_init(void);
extern inline void usb_renumber(void);
extern void usb_handle_setup_data(void);

#endif
