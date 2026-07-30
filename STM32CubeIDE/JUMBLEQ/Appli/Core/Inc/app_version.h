#ifndef INC_APP_VERSION_H_
#define INC_APP_VERSION_H_

#include <stdint.h>

#define APP_VERSION_MAJOR 0
#define APP_VERSION_MINOR 9
#define APP_VERSION_PATCH 12

#define APP_VERSION_STR_HELPER(x) #x
#define APP_VERSION_STR_VALUE(x)  APP_VERSION_STR_HELPER(x)

#define APP_VERSION_STR      APP_VERSION_STR_VALUE(APP_VERSION_MAJOR) "." APP_VERSION_STR_VALUE(APP_VERSION_MINOR) "." APP_VERSION_STR_VALUE(APP_VERSION_PATCH)
#define APP_VERSION_OLED_STR "ver" APP_VERSION_STR

/* USB bcdDevice layout: major.minor.patch = M.m.pp (one BCD digit, one BCD digit, two BCD digits). */
#if (APP_VERSION_MAJOR > 9) || (APP_VERSION_MINOR > 9) || (APP_VERSION_PATCH > 99)
#error "APP_VERSION cannot be represented by APP_VERSION_USB_BCD"
#endif

#define APP_VERSION_USB_BCD ((uint16_t) (((APP_VERSION_MAJOR) << 12) | \
                                         ((APP_VERSION_MINOR) << 8) | \
                                         (((APP_VERSION_PATCH) / 10) << 4) | \
                                         ((APP_VERSION_PATCH) % 10)))

#endif /* INC_APP_VERSION_H_ */
