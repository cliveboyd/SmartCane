/* Copyright (c) <2015> <Shun Bai (wanyancan at gmail)>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

 */
 
#ifndef SERVICE_MAKER_H__
#define SERVICE_MAKER_H__

#include <stdint.h>

//#define MAKE_FN_NAME(x) void  Callback_ ## x (void)
//#define FUNCTION_NAME(signal) MAKE_FN_NAME(signal)

#define MAKE_UUID_BASE_(SERVICEName) static unsigned char SERVICEName ## _UUID_BASE[]
#define MAKE_UUID_BASE(x) MAKE_UUID_BASE_(x)

#define MAKE_UUID_SERVICE_(SERVICEName) SERVICEName ## _UUID_SERVICE
#define MAKE_UUID_SERVICE(x,value) MAKE_UUID_SERVICE_(x) = value

#define MAKE_UUID_CHAR_(SERVICEName,CHARName) SERVICEName ## _UUID_ ## CHARName ## _CHAR
#define MAKE_UUID_CHAR(servname,charname,value) MAKE_UUID_CHAR_(servname,charname) = value
#define MAKE_UUID_CHAR_NOVALUE(servname,charname) MAKE_UUID_CHAR_(servname,charname)

#define MAKE_UUID_ENUM_(SERVICEName) SERVICEName ## _UUID_t
#define MAKE_UUID_ENUM(servname) MAKE_UUID_ENUM_(servname)

#define MAKE_SERVICE_STRUCT_(SERVICEName) typedef struct ble_ ## SERVICEName ## _s ble_ ## SERVICEName ## _t
#define MAKE_SERVICE_STRUCT(servname) MAKE_SERVICE_STRUCT_(servname)
#define MAKE_SERVICE_STRUCT_NAME_(SERVICEName)  ble_ ## SERVICEName ## _t
#define MAKE_SERVICE_STRUCT_NAME(servname) MAKE_SERVICE_STRUCT_NAME_(servname)

#define MAKE_CHAR_HANDLEID_(CHARname) CHARname ## _char_handleID
#define MAKE_CHAR_HANDLEID(charname) MAKE_CHAR_HANDLEID_(charname)
#define MAKE_SERVICE_STRUCT_CONTENT_(SERVICEName,content) typedef struct ble_ ## SERVICEName ## _s content ble_ ## SERVICEName ## _t
#define MAKE_SERVICE_STRUCT_CONTENT(servname,con) MAKE_SERVICE_STRUCT_CONTENT_(servname,con)

#define MAKE_SERVICE_INIT_STRUCT_(SERVICEName) typedef struct ble_ ## SERVICEName ## _init_s ble_ ## SERVICEName ## _init_t
#define MAKE_SERVICE_INIT_STRUCT(servname) MAKE_SERVICE_INIT_STRUCT_(servname)
#define MAKE_SERVICE_INIT_STRUCT_CONTENT_(SERVICEName,content) typedef struct ble_ ## SERVICEName ## _init_s content ble_ ## SERVICEName ## _init_t
#define MAKE_SERVICE_INIT_STRUCT_CONTENT(servname,con) MAKE_SERVICE_INIT_STRUCT_CONTENT_(servname,con)

#define MAKE_SERVICE_INIT_CALL_(SERVICEName) uint32_t ble_ ## SERVICEName ## _init(ble_ ## SERVICEName ## _t * , const ble_ ## SERVICEName ## _init_t * )
#define MAKE_SERVICE_INIT_CALL(servname) MAKE_SERVICE_INIT_CALL_(servname)

#define MAKE_SERVICE_BLE_EVT_HANDLER_(SERVICEName) void ble_ ## SERVICEName ## _on_ble_evt(ble_ ## SERVICEName ## _t * , ble_evt_t * )
#define MAKE_SERVICE_BLE_EVT_HANDLER(servname) MAKE_SERVICE_BLE_EVT_HANDLER_(servname)
#endif // SERVICE_MAKER_H__
