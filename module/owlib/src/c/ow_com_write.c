/*
$Id$
    OWFS -- One-Wire filesystem
    OWHTTPD -- One-Wire Web Server
    Written 2003 Paul H Alfille
	email: palfille@earthlink.net
	Released under the GPL
	See the header file: ow.h for full attribution
	1wire/iButton system from Dallas Semiconductor
*/

#include <config.h>
#include "owfs_config.h"
#include "ow.h"
#include "ow_connection.h"

#ifdef HAVE_LINUX_LIMITS_H
#include <linux/limits.h>
#endif

int COM_write( const char * data, size_t length, const struct parsedname * pn )
{
	struct connection_in * connection = pn->selected_connection ;
	ssize_t to_be_written = length ;

	if ( length == 0 || data == NULL ) {
		return 0 ;
	}

	if ( connection == NULL ) {
		return -EIO ;
	}

	while (to_be_written > 0) {
		int select_result ;

		fd_set writeset;
		struct timeval tv;

		// use same timeout as read as for write
		tv.tv_sec = Globals.timeout_serial;
		tv.tv_usec = 0;
		/* Initialize readset */
		FD_ZERO(&writeset);
		FD_SET(connection->file_descriptor, &writeset);

		/* Read if it doesn't timeout first */
		select_result = select(connection->file_descriptor + 1, NULL, &writeset, NULL, &tv);
		if (select_result > 0) {
			ssize_t write_result ;

			if (FD_ISSET(connection->file_descriptor, &writeset) == 0) {
				STAT_ADD1_BUS(e_bus_write_errors, connection);
				return -EIO;	/* error */
			}
			update_max_delay(pn);
			Debug_Bytes("Attempt serial write:",  &data[length - to_be_written], to_be_written);
			write_result = write(connection->file_descriptor, &data[length - to_be_written], to_be_written);	/* write bytes */
			if (write_result < 0) {
				if (errno != EWOULDBLOCK) {
					ERROR_CONNECT("Trouble writing to serial port: %s\n", SAFESTRING(connection->name));
					STAT_ADD1_BUS(e_bus_write_errors, connection);
					return write_result;
				}
				/* write() was interrupted, try again */
				STAT_ADD1_BUS(e_bus_timeouts, connection);
			} else {
				to_be_written -= write_result ;	
			}
		} else {			/* timed out or select error */
			STAT_ADD1_BUS(e_bus_timeouts, connection);
			return -errno;
		}
	}

	tcdrain(connection->file_descriptor);
	gettimeofday(&(connection->bus_write_time), NULL);

	return to_be_written ;
}