#include <cmd/cmd_op_codes.h>
#include <stdint.h>
#include <stddef.h>

#include "queue.h"
#include "cmd_op_codes.h"

#ifndef CMD_H_
#define CMD_H_

/**
 * Data to pass in when processing the indicated command.
 */
struct cmd_data {
	/**
	 * The number of bytes in payload.
	 */
	uint8_t len;
	/**
	 * Pointer to the payload data.
	 */
	uint8_t *payload;
};

/**
 * 'Write' callback prototype for a command in the command parser.
 *
 * @param op_code 	The op_code associated with this command callback
 * @param pdata 	The data to use when parsing this command
 * @param arg		Pointer to an optional user-defined argument.
 *
 * @return 0 if everything executed correctly, otherwise an error code.
 */
typedef int (*cmd_write_cb_t)(uint8_t op_code,
		     struct cmd_data *pdata, void *arg);

/**
 * 'Read' callback prototype for a command in the command parser.
 *
 * @param op_code 	The op_code associated with this command callback
 * @param pdata 	The response data to return for this command
 * @param arg		Pointer to an optional user-defined argument.
 *
 * @return 0 if everything executed correctly, otherwise an error code.
 */
typedef int (*cmd_read_cb_t)(uint8_t op_code,
		     struct cmd_data *pdata, void *arg);

/**
 * Settings for a single command record for registration and parsing in the
 * command parsing engine.
 */
struct cmd_record {
	/**
	 * The unique 8-bit opcode to associate with this command record.
	 */
	uint8_t op_code;

	/**
	 * Pointer to the cmd_write_cb_t callback function to fire for this record.
	 */
	cmd_write_cb_t write;

	/**
	 * Pointer to the cmd_read_cb_t callback function to fire for this record.
	 */
	cmd_write_cb_t read;

	/**
	 * Timeout in ticks for this command to finish execution (0=infinity).
	 */
	uint32_t timeout;

    /**
     * The next record in the global command record list.
     */
    SLIST_ENTRY(cmd_record) s_next;
};

/**
 * Initialises the command parser.
 *
 * @return 0 if everything executed correctly, otherwise an error code.
 */
int cmd_init(void);

/**
* Adds a new command to the command registry
*
* @param rec The new record to register.
*
* @return 0 if everything executed correctly, otherwise an error code.
*/
int cmd_register(struct cmd_record *rec);

/**
 * Attempts to parse a command using the specified op_code, data and
 * an optional user-defined argument (if necessary).
 *
 * @param op_code	The unique 8-bit op code
 * @param pdata		Pointer to the payload associated with this op code
 * @param arg		Pointer to an optional user-defined argument, which
 *                  will be passed to the command's callback handler.
 *
 * @return 0 if everything executed correctly, otherwise an error code.
 */
int cmd_write(uint8_t op_code, struct cmd_data *pdata, void *arg);

/**
 * Attempts to read a response associated with the specified op_code.
 *
 * @param op_code	The unique 8-bit op code to get a response from.
 * @param pdata		Pointer to the payload associated with this op code
 * @param arg		Pointer to an optional user-defined argument, which
 *                  will be passed to the command's callback handler.
 *
 * @return 0 if everything executed correctly, otherwise an error code.
 */
int cmd_read(uint8_t op_code, struct cmd_data *pdata, void *arg);

#endif /* CMD_H_ */
