#ifndef SRC_FLEXPTP_OPTIONS_STM32F407_ETHERLIB
#define SRC_FLEXPTP_OPTIONS_STM32F407_ETHERLIB

#ifdef ETH_ETHERLIB
#define ETHLIB
#elif defined(ETH_LWIP)
#define LWIP
#endif

#define FLEXPTP_TASK_PRIORITY 24

// -------------------------------------------
// --- DEFINES FOR PORTING IMPLEMENTATION ----
// -------------------------------------------

// Include LwIP/EtherLib headers here

#ifdef ETH_ETHERLIB
#include <etherlib/etherlib.h>
#elif defined(ETH_LWIP)
#include <lwip/ip_addr.h>
#endif

// Give a printf-like printing implementation MSG(...)
// Give a maskable printing implementation CLILOG(en,...)
// Provide an SPRINTF-implementation SPRINTF(str,n,fmt,...)

#include "cliutils/cli.h"
#include "embfmt/embformat.h"
#include "standard_output/standard_output.h"

#define FLEXPTP_SNPRINTF(...) embfmt(__VA_ARGS__)

// Include hardware port files and fill the defines below to port the PTP stack to a physical hardware:
// - PTP_HW_INIT(increment, addend): function initializing timestamping hardware
// - PTP_MAIN_OSCILLATOR_FREQ_HZ: clock frequency fed into the timestamp unit [Hz]
// - PTP_INCREMENT_NSEC: hardware clock increment [ns]
// - PTP_UPDATE_CLOCK(s,ns): function jumping clock by defined value (negative time value means jumping backward)
// - PTP_SET_ADDEND(addend): function writing hardware clock addend register

#include "Drivers/EthDrv/mac_drv.h"

#ifdef ETH_ETHERLIB
#include <flexptp/port/example_ports/ptp_port_stm32f407_etherlib.h>
#elif defined(ETH_LWIP)
#include <flexptp/port/example_ports/ptp_port_stm32f407_lwip.h>
#endif

#define PTP_MAIN_OSCILLATOR_FREQ_HZ (168000000)
#define PTP_INCREMENT_NSEC (6)

#include <stdlib.h>

#define PTP_HW_INIT(increment, addend) ptphw_init(increment, addend)
#define PTP_UPDATE_CLOCK(s, ns) ETHHW_UpdatePTPTime(ETH, labs(s), abs(ns), (s * NANO_PREFIX + ns) < 0)
#define PTP_SET_CLOCK(s, ns) ETHHW_InitPTPTime(ETH, labs(s), abs(ns))
#define PTP_SET_ADDEND(addend) ETHHW_SetPTPAddend(ETH, addend)
#define PTP_HW_GET_TIME(pt) ptphw_gettime(pt)

// Include the clock servo (controller) and define the following:
// - PTP_SERVO_INIT(): function initializing clock servo
// - PTP_SERVO_DEINIT(): function deinitializing clock servo
// - PTP_SERVO_RESET(): function reseting clock servo
// - PTP_SERVO_RUN(d): function running the servo, input: master-slave time difference (error), return: clock tuning value in PPB
//

#include <flexptp/servo/pid_controller.h>

#define PTP_SERVO_INIT() pid_ctrl_init()
#define PTP_SERVO_DEINIT() pid_ctrl_deinit()
#define PTP_SERVO_RESET() pid_ctrl_reset()
#define PTP_SERVO_RUN(d, pscd) pid_ctrl_run(d, pscd)

// Optionally add interactive, tokenizing CLI-support
// - CLI_REG_CMD(cmd_hintline,n_cmd,n_min_arg,cb): function for registering CLI-commands
//      cmd_hintline: text line printed in the help beginning with the actual command, separated from help text by \t charaters
//      n_cmd: number of tokens (words) the command consists of
//      n_arg: minimal number of arguments must be passed with the command
//      cb: callback function cb(const CliToken_Type *ppArgs, uint8_t argc)
//  return: cmd id (can be null, if discarded)

#define CLI_REG_CMD(cmd_hintline, n_cmd, n_min_arg, cb) cli_register_command(cmd_hintline, n_cmd, n_min_arg, cb)

// -------------------------------------------

#define CLILOG(en, ...)       \
    {                         \
        if (en)               \
            MSG(__VA_ARGS__); \
    }

// -------------------------------------------

#define PTP_ENABLE_MASTER_OPERATION (1) // Enable or disable master operations

// Static fields of the Announce message
#define PTP_CLOCK_PRIORITY1 (128)                      // priority1 (0-255)
#define PTP_CLOCK_PRIORITY2 (128)                      // priority2 (0-255)
#define PTP_BEST_CLOCK_CLASS (PTP_CC_DEFAULT)          // best clock class of this device
#define PTP_WORST_ACCURACY (PTP_CA_UNKNOWN)            // worst accucary of the clock
#define PTP_TIME_SOURCE (PTP_TSRC_INTERNAL_OSCILLATOR) // source of the distributed time

// -------------------------------------------

#include "flexptp/event.h"
extern void flexptp_user_event_cb(PtpUserEventCode uev);
#define PTP_USER_EVENT_CALLBACK flexptp_user_event_cb

#endif /* SRC_FLEXPTP_OPTIONS_STM32F407_ETHERLIB */
