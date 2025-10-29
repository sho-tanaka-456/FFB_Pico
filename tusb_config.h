#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUSB_MCU             OPT_MCU_RP2040
#define CFG_TUSB_RHPORT0_MODE    OPT_MODE_DEVICE
//#define CFG_TUSB_OS              OPT_OS_NONE

// メモリ制限（Picoはスタックに制限あり）
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN       __attribute__ ((aligned(4)))

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUD_ENDPOINT0_SIZE    64

// 利用するクラス
#define CFG_TUD_HID               1  // HIDを1つ使う
#define CFG_TUD_HID_EP_BUFSIZE    64

//--------------------------------------------------------------------
// HID設定
//--------------------------------------------------------------------



#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
