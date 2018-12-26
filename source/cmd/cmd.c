#include <stdio.h>
#include "cmd.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "cmd/cmd_whoami.h"
#include "cmd/cmd_version.h"
#include "cmd/cmd_adc.h"

/**
 * Debug stats for the command parsing engine.
 */
volatile struct {
	uint32_t registers;		/**< Number of commands inserted into the registry. */
	uint32_t writes;		/**< Number of command write requests. */
	uint32_t reads;			/**< Number of command read requests. */
	uint32_t inits;			/**< Number of init requests. */
	uint32_t err_ocds;	    /**< Number of unsupported opcode requests. */
} g_cmd_stats = { 0 };

/**
 * The command registry that all op-codes will be parsed against.
 * Implemented as a singly-linked list.
 */
struct {
    SLIST_HEAD(, cmd_record) mgr_cmd_list;
} cmd_registry_mgr;

/**
 * Insert a new command record into the command registry.
 *
 * @param rec	The cmd_record to insert in the command registry.
 */
static void
cmd_mgr_insert(struct cmd_record *rec)
{
    SLIST_INSERT_HEAD(&cmd_registry_mgr.mgr_cmd_list, rec, s_next);
}

int
cmd_register(struct cmd_record *rec)
{
	g_cmd_stats.registers++;
    //printf("Registering command 0x%02X\n", rec->op_code);
    //fflush(stdout);
    cmd_mgr_insert(rec);

    return kStatus_Success;
}

int
cmd_write(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
    struct cmd_record *cursor;

    cursor = NULL;
    SLIST_FOREACH(cursor, &cmd_registry_mgr.mgr_cmd_list, s_next) {
        if (cursor->op_code == op_code) {
            g_cmd_stats.writes++;
            /* Forward the access request to the right command handler */
            return cursor->write(op_code, pdata, arg);
        }
    }

    /* No matching op-code */
    g_cmd_stats.err_ocds++;
    return kStatus_InvalidArgument;
}

int
cmd_read(uint8_t op_code, struct cmd_data *pdata, void *arg)
{
    struct cmd_record *cursor;

    cursor = NULL;
    SLIST_FOREACH(cursor, &cmd_registry_mgr.mgr_cmd_list, s_next) {
        if (cursor->op_code == op_code) {
            g_cmd_stats.reads++;
            /* Forward the access request to the right command handler */
            return cursor->read(op_code, pdata, arg);
        }
    }

    /* No matching op-code */
    g_cmd_stats.err_ocds++;
    return kStatus_InvalidArgument;
}

int
cmd_init(void)
{
	int rc;

	/* TODO: Add locking mechanism on cmd_registry_mgr. */

    g_cmd_stats.inits++;

	/* Add the WHOAMI opcode */
	rc = cmd_whoami_init();
	if (rc) {
		goto err;
	}

	/* Add the VERSION opcode */
	rc = cmd_version_init();
	if (rc) {
		goto err;
	}

	/* Add the ADC opcodes */
	rc = cmd_adc_init();
	if (rc) {
		goto err;
	}

	return kStatus_Success;
err:
	return rc;
}
