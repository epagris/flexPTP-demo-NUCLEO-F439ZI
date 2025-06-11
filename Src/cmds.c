#include "cmds.h"

#include <FreeRTOS.h>
#include <task.h>

#include "cliutils/cli.h"
#include "standard_output/standard_output.h"

#include "flexptp/task_ptp.h"

// ---------------------------------

CMD_FUNCTION(os_info) {
    MSG("OS: FreeRTOS\nKernel version: %s\n", tskKERNEL_VERSION_NUMBER);

    HeapStats_t stats;
    vPortGetHeapStats(&stats);
    MSG("Free OS memory: %u bytes\n", stats.xAvailableHeapSpaceInBytes);

    return 0;
}

CMD_FUNCTION(start_flexptp) {
    if (!task_ptp_is_operating()) {
        MSG("Starting flexPTP...\n\n");
        reg_task_ptp();
    } else {
        MSG("Nice try, but no. :)\n"
            "flexptp is already up and running!\n");
    }

    return 0;
}

// ---------------------------------

void cmd_init() {
    cli_register_command("osinfo \t\t\tPrint OS-related information", 1, 0, os_info);
    cli_register_command("flexptp \t\t\tStart flexPTP daemon", 1, 0, start_flexptp);
}