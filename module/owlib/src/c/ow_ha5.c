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
#include "ow_counters.h"
#include "ow_connection.h"
#include "ow_codes.h"

//static void byteprint( const BYTE * b, int size ) ;
static int HA5_reset(const struct parsedname *pn);
static int HA5_reset_wrapped(const struct parsedname *pn) ;
static int HA5_next_both(struct device_search *ds, const struct parsedname *pn);
static int HA5_sendback_part(char cmd, const BYTE * data, BYTE * resp, const size_t size, const struct parsedname *pn) ;
static int HA5_sendback_data(const BYTE * data, BYTE * resp, const size_t len, const struct parsedname *pn);
static int HA5_select_and_sendback(const BYTE * data, BYTE * resp, const size_t len, const struct parsedname *pn);
static void HA5_setroutines(struct connection_in *in);
static void HA5_close(struct connection_in *in);
static int HA5_directory(struct device_search *ds, struct dirblob *db, const struct parsedname *pn);
static int HA5_select( const struct parsedname * pn ) ;
static int HA5_select_wrapped( const struct parsedname * pn ) ;
static int HA5_resync( const struct parsedname * pn ) ;
static void HA5_powerdown(struct connection_in * in) ;
static int Parse_first_ha5_address( struct connection_in * in ) ;
static char Parse_next_ha5_address( char * name ) ;
static int HA5_test_channel( struct parsedname  *pn ) ;
static int AddChecksum( unsigned char * check_string, int length, struct connection_in * in ) ;
static int TestChecksum( unsigned char * check_string, int length ) ;
static int HA5_find_channel(struct parsedname *pn) ;

static void HA5_setroutines(struct connection_in *in)
{
	in->iroutines.detect = HA5_detect;
	in->iroutines.reset = HA5_reset;
	in->iroutines.next_both = HA5_next_both;
	in->iroutines.PowerByte = NULL;
//    in->iroutines.ProgramPulse = ;
	in->iroutines.sendback_data = HA5_sendback_data;
	in->iroutines.select_and_sendback = HA5_select_and_sendback;
	//    in->iroutines.sendback_bits = ;
	in->iroutines.select = HA5_select ;
	in->iroutines.reconnect = NULL;
	in->iroutines.close = HA5_close;
	in->iroutines.transaction = NULL;
	in->iroutines.flags = ADAP_FLAG_dirgulp | ADAP_FLAG_bundle | ADAP_FLAG_dir_auto_reset;
	in->bundling_length = HA5_FIFO_SIZE;
}

int HA5_detect(struct connection_in *in)
{
	struct parsedname pn;
	
	FS_ParsedName(NULL, &pn);	// minimal parsename -- no destroy needed
	pn.selected_connection = in;
	
	/* Set up low-level routines */
	HA5_setroutines(in);
	
	/* Initialize dir-at-once structures */
	DirblobInit(&(in->connin.ha5.main));
	DirblobInit(&(in->connin.ha5.alarm));
	
	// Poison current "Address" for adapter
	in->connin.ha5.sn[0] = 0 ; // so won't match
	
	/* Open the com port */
	if (COM_open(in)) {
		return -ENODEV;
	}
	
	in->connin.ha5.checksum = Globals.checksum ;
	in->Adapter = adapter_HA5 ;
	in->adapter_name = "HA5";

	/* By definition, this is the head adapter on this port */
	in->connin.ha5.head = in ;
#if OW_MT
	my_pthread_mutex_init(&(in->connin.ha5.lock), Mutex.pmattr);
#endif							/* OW_MT */

	/* Find the channels */
	if ( Parse_first_ha5_address(in) ) {
		if ( HA5_find_channel(&pn) ) {
			HA5_close(in) ;
			return -ENODEV ;
		}
	} else { // A list of channels
		char next_char ;
		while ( (next_char = Parse_next_ha5_address(in->name)) ) {
			struct connection_in * added = NewIn(in) ;
			if ( added == NULL ) {
				break ;
			}
			added->connin.ha5.channel = next_char ;
			pn.selected_connection = added ;
			HA5_reset(&pn) ;
		}
	}

	COM_slurp(in->file_descriptor) ;
	
	pn.selected_connection = in;
	HA5_reset(&pn) ;
	return 0;
}

/* Search for a ":" in the name
Change it to a null,and parse the remaining text as a letter
return 0 -- success, else 1
NOTE: Alters in->name
*/
static int Parse_first_ha5_address( struct connection_in * in )
{
	char * colon = strchr( in->name, ':' ) ;
	if ( colon == NULL ) { // not found
		return 1 ;
	}
	
	colon[0] = 0x00 ; // set as null
	
	if ( isalpha( colon[1] ) ) {
		in->connin.ha5.channel = tolower( colon[1] ) ;
		colon[1] = ':' ;
		return  0 ;
	}
	return 1 ;
}

static char Parse_next_ha5_address( char * name )
{
	char * colon ;
	char * post_name ;
	char return_char ;

	post_name = & name[strlen(name)+1] ;
	colon = & post_name[strspn(post_name,":")] ;
	if ( colon[0] == '\0' ) { // not found
		return 0 ;
	}
	
	if ( isalpha( colon[0] ) ) {
		return_char = tolower( colon[0] ) ;
		colon[0] = ':' ;
		return  return_char ;
	}
	return 0 ;
}

/* Find the HA5 channel (since none specified) */
/* Arbitrarily assign it to "a" if none found */
static int HA5_find_channel(struct parsedname *pn)
{
	struct connection_in * in = pn->selected_connection ;

	for ( in->connin.ha5.channel = 'a' ; in->connin.ha5.channel <= 'z' ; ++in->connin.ha5.channel ) {
		if ( HA5_test_channel(pn) == 0 ) {
			LEVEL_DEBUG("HA5 adapter found on port %s at channel %c\n", in->name, in->connin.ha5.channel ) ;
			return 0 ;
		}
	}
	in->connin.ha5.channel = 'a' ;
	LEVEL_DEBUG("HA5 adapter not found on port %s so set to channel %c\n", in->name, in->connin.ha5.channel ) ;
	
	return 0;
}

static int AddChecksum( unsigned char * check_string, int length, struct connection_in * in )
{
	if ( in->connin.ha5.checksum ) {
		int i ;
		unsigned char sum = 0 ;
		for ( i=0 ; i<length ; ++i ) {
			sum += check_string[i] ;
		}
		num2string( (char *) & check_string[length], sum & 0xFF ) ;
		check_string[length+2] = 0x0D ;
		return length+3 ;
	} else {
		check_string[length] = 0x0D ;
		return length+1 ;
	}
}

static int TestChecksum( unsigned char * check_string, int length )
{
	int i ;
	unsigned char sum = 0 ;
	for ( i=0 ; i<length ; ++i ) {
		sum += check_string[i] ;
	}
	if ( string2num( (char *) & check_string[length]) == (sum & 0xFF) ) {
		return 0 ;
	}
	return 1 ;
}

static int HA5_test_channel( struct parsedname  *pn )
{
	unsigned char test_string[1+1+2+1] ;
	unsigned char test_response[1] ;
	int string_length ;
	struct connection_in * in = pn->selected_connection ;

	COM_slurp(in->file_descriptor) ;

	test_string[0] = in->connin.ha5.channel ;
	test_string[1] = 'R' ;
	string_length = AddChecksum( test_string, 2, in ) ;

	if ( COM_write( test_string, string_length, pn ) ) {
		return 1 ;
	}
	if ( COM_read( test_response, 1, pn ) ) {
		return 1 ;
	}
	
	COM_slurp(in->file_descriptor) ;
	return 0 ;
}

static int HA5_reset(const struct parsedname *pn)
{
	struct connection_in * in = pn->selected_connection ;
	int ret ;

	pthread_mutex_lock( &in->connin.ha5.head->connin.ha5.lock ) ;
	ret = HA5_reset_wrapped(pn) ;
	pthread_mutex_unlock( &in->connin.ha5.head->connin.ha5.lock ) ;

	return ret ;
}

static int HA5_reset_wrapped(const struct parsedname *pn)
{
	struct connection_in * in = pn->selected_connection ;
	int reset_length ;
	BYTE reset[5] ;
	BYTE resp[4];
	
	reset[0] = in->connin.ha5.channel ;
	reset[1] = 'R' ;
	reset_length = AddChecksum( reset, 2, in ) ;
	
	if (COM_write(reset, reset_length, pn)) {
		LEVEL_DEBUG("Error sending HA5 reset\n");
		return -EIO;
	}
	if ( in->connin.ha5.checksum ) {
		if (COM_read(resp, 4, pn)) {
			LEVEL_DEBUG("Error reading HA5 reset\n");
			return -EIO;
		}
		if ( TestChecksum( resp, 1 ) ) {
			LEVEL_DEBUG("HA5 reset checksum error\n");
			return -EIO;
		}
	} else {
		if (COM_read(resp, 2, pn)) {
			LEVEL_DEBUG("Error reading HA5 reset\n");
			return -EIO;
		}
	}
		
	switch( resp[0] ) {
		case 'P':
			in->AnyDevices = 1 ;
			break ;
		case 'N':
			in->AnyDevices = 0 ;
			break ;
		default:
			LEVEL_DEBUG("Error HA5 reset bad response %c (0x%.2X)\n", resp[0], resp[0]);
			return -EIO;
	}
	return BUS_RESET_OK;
}

static int HA5_next_both(struct device_search *ds, const struct parsedname *pn)
{
	int ret = 0;
	struct connection_in * in = pn->selected_connection ;
	struct dirblob *db = (ds->search == _1W_CONDITIONAL_SEARCH_ROM) ?
		&(in->connin.link.alarm) : &(in->connin.link.main);

	if (ds->LastDevice) {
		return -ENODEV;
	}

	COM_flush(pn);

	if (ds->index == -1) {

		pthread_mutex_lock( &in->connin.ha5.head->connin.ha5.lock ) ;
		ret = HA5_directory(ds, db, pn) ;
		pthread_mutex_unlock( &in->connin.ha5.head->connin.ha5.lock ) ;

		if ( ret ) {
			return -EIO;
		}
	}
	// LOOK FOR NEXT ELEMENT
	++ds->index;

	LEVEL_DEBUG("Index %d\n", ds->index);

	ret = DirblobGet(ds->index, ds->sn, db);
	LEVEL_DEBUG("DirblobGet %d\n", ret);
	switch (ret) {
	case 0:
		if ((ds->sn[0] & 0x7F) == 0x04) {
			/* We found a DS1994/DS2404 which require longer delays */
			pn->selected_connection->ds2404_compliance = 1;
		}
		break;
	case -ENODEV:
		ds->LastDevice = 1;
		break;
	}

	LEVEL_DEBUG("HA5_next_both SN found: " SNformat "\n", SNvar(ds->sn));
	return ret;
}

/************************************************************************/
/*	HA5_directory: searches the Directory stores it in a dirblob	     */
/*			& stores in in a dirblob object depending if it              */
/*			Supports conditional searches of the bus for 	             */
/*			/alarm branch					                             */
/*                                                                       */
/* Only called for the first element, everything else comes from dirblob */
/* returns 0 even if no elements, errors only on communication errors    */
/************************************************************************/
static int HA5_directory(struct device_search *ds, struct dirblob *db, const struct parsedname *pn)
{
	unsigned char resp[20];
	unsigned char query[5+2+1] ;
	int query_length ;
	struct connection_in * in = pn->selected_connection ;

	DirblobClear(db);

	//Depending on the search type, the HA5 search function
	//needs to be selected
	//tEC -- Conditional searching
	//tF0 -- Normal searching

	// Send the configuration command and check response
	query[0] = in->connin.ha5.channel ;
	query[1] = (ds->search == _1W_CONDITIONAL_SEARCH_ROM) ? 'C' : 'S' ;
	query[2] = ',' ;
	query[3] = 'F' ;
	query[4] = 'F' ;
	query_length = AddChecksum( query, 5, in ) ;

	if (COM_write( query, query_length, pn)) {
		return HA5_resync(pn) ;
	}
	
	if (COM_read(resp, 1, pn)) {
		return HA5_resync(pn) ;
	}

	while ( resp[0] != 0x0D ) {
		BYTE sn[8];
		char wrap_char ;
		//One needs to check the first character returned.
		//If nothing is found, the ha5 will timeout rather then have a quick
		//return.  This happens when looking at the alarm directory and
		//there are no alarms pending
		//So we grab the first character and check it.  If not an E leave it
		//in the resp buffer and get the rest of the response from the HA5
		//device

		if ( in->connin.ha5.checksum ) {
			if (COM_read(&resp[1], 19, pn)) {
				return HA5_resync(pn) ;
			}
			if ( resp[18]!=0x0D ) {
				return HA5_resync(pn) ;
			}
			wrap_char = resp[19] ;
			if ( TestChecksum( resp, 16 ) ) {
				return HA5_resync(pn) ;
			}
		} else {
			if (COM_read(&resp[1], 17, pn)) {
				return HA5_resync(pn) ;
			}
			if ( resp[16]!=0x0D ) {
				return HA5_resync(pn) ;
			}
			wrap_char = resp[17] ;
		}
		sn[7] = string2num((char *)&resp[0]);
		sn[6] = string2num((char *)&resp[2]);
		sn[5] = string2num((char *)&resp[4]);
		sn[4] = string2num((char *)&resp[6]);
		sn[3] = string2num((char *)&resp[8]);
		sn[2] = string2num((char *)&resp[10]);
		sn[1] = string2num((char *)&resp[12]);
		sn[0] = string2num((char *)&resp[14]);

		// Set as current "Address" for adapter
		memcpy( pn->selected_connection->connin.ha5.sn, sn, 8) ;

		LEVEL_DEBUG("HA5_directory SN found: " SNformat "\n", SNvar(sn));
		// CRC check
		if (CRC8(sn, 8) || (sn[0] == 0)) {
			/* A minor "error" and should perhaps only return -1 */
			/* to avoid reconnect */
			LEVEL_DEBUG("sn = %s\n", sn);
			return HA5_resync(pn) ;
		}
		DirblobAdd(sn, db);
		resp[0] = wrap_char ;
	}
	return 0 ;
}

static int HA5_resync( const struct parsedname * pn )
{
	COM_flush(pn);
	HA5_reset(pn);
	COM_flush(pn);

	// Poison current "Address" for adapter
	pn->selected_connection->connin.ha5.sn[0] = 0 ; // so won't match

	return -EIO ;
}

static int HA5_select( const struct parsedname * pn )
{
	struct connection_in * in = pn->selected_connection ;
	int ret ;

	if ( (pn->selected_device==NULL) || (pn->selected_device==DeviceThermostat) ) {
		return HA5_reset(pn) ;
	}
	
	pthread_mutex_lock( &in->connin.ha5.head->connin.ha5.lock ) ;
	ret = HA5_select_wrapped(pn) ;
	pthread_mutex_unlock( &in->connin.ha5.head->connin.ha5.lock ) ;

	return ret ;
}

static int HA5_select_wrapped( const struct parsedname * pn )
{
	struct connection_in * in = pn->selected_connection ;
	unsigned char send_address[21] ;
	unsigned char resp_address[19] ;
	int send_length ;
	
	send_address[0] = in->connin.ha5.channel ;
	send_address[1] = 'A' ;
	num2string( (char *)&send_address[ 2], pn->sn[7] ) ;
	num2string( (char *)&send_address[ 4], pn->sn[6] ) ;
	num2string( (char *)&send_address[ 6], pn->sn[5] ) ;
	num2string( (char *)&send_address[ 8], pn->sn[4] ) ;
	num2string( (char *)&send_address[10], pn->sn[3] ) ;
	num2string( (char *)&send_address[12], pn->sn[2] ) ;
	num2string( (char *)&send_address[14], pn->sn[1] ) ;
	num2string( (char *)&send_address[16], pn->sn[0] ) ;

	send_length = AddChecksum( send_address, 18, in ) ;
	
	if ( COM_write( send_address, send_length, pn) ) {
		LEVEL_DEBUG("Error with sending HA5 A-ddress\n") ;
		return HA5_resync(pn) ;
	}

	if ( in->connin.ha5.checksum ) {
		if ( COM_read(resp_address,19,pn) ) {
			LEVEL_DEBUG("Error with reading HA5 select\n") ;
			return HA5_resync(pn) ;
		}
		if ( TestChecksum( resp_address, 16) ) {
			LEVEL_DEBUG("HA5 select checksum error\n") ;
			return HA5_resync(pn) ;
		}
	} else {
		if ( COM_read(resp_address,17,pn) ) {
			LEVEL_DEBUG("Error with reading HA5 select\n") ;
			return HA5_resync(pn) ;
		}
	}
	if ( memcmp( &resp_address[0],&send_address[2],16) ) {
		LEVEL_DEBUG("Error with HA5 select response\n") ;
		return HA5_resync(pn) ;
	}
	
	// Set as current "Address" for adapter
	memcpy( in->connin.ha5.sn, pn->sn, 8) ;
	
	return 0 ;
}

//  Send data and return response block -- up to 32 bytes
static int HA5_sendback_part(char cmd, const BYTE * data, BYTE * resp, const size_t size, const struct parsedname *pn)
{
	struct connection_in * in = pn->selected_connection ;
	unsigned char send_data[2+2+32*2+3] ;
	unsigned char get_data[32*2+3] ;
	int send_length ;

	send_data[0] = in->connin.ha5.channel ;
	send_data[1] = cmd ;
	num2string( (char *)&send_data[2], size ) ;
	bytes2string( (char *)&send_data[4], data, size) ;
	send_length = AddChecksum( send_data, 4+size*2, in ) ;

	if ( COM_write( send_data, send_length, pn) ) {
		LEVEL_DEBUG("Error with sending HA5 block\n") ;
		return HA5_resync(pn) ;
	}

	if ( in->connin.ha5.checksum ) {
		if ( COM_read( get_data, size*2+3, pn) ) {
			LEVEL_DEBUG("Error with reading HA5 block\n") ;
			return HA5_resync(pn) ;
		}
		if ( TestChecksum( get_data, size*2) ) {
			LEVEL_DEBUG("HA5 block read checksum error\n") ;
			return HA5_resync(pn) ;
		}
	} else {
		if ( COM_read( get_data, size*2+1, pn) ) {
			LEVEL_DEBUG("Error with reading HA5 block\n") ;
			return HA5_resync(pn) ;
		}
	}
	string2bytes( (char *)get_data, resp, size) ;
	return 0 ;
}

static int HA5_sendback_data(const BYTE * data, BYTE * resp, const size_t size, const struct parsedname *pn)
{
	struct connection_in * in = pn->selected_connection ;
	int left;
	
	for ( left=size ; left>0 ; left -= 32 ) {
		int ret ;
		size_t pass_start = size - left ;
		size_t pass_size = (left>32)?32:left ;
		
		pthread_mutex_lock( &in->connin.ha5.head->connin.ha5.lock ) ;
		ret = HA5_sendback_part( 'W', &data[pass_start], &resp[pass_start], pass_size, pn ) ;
		pthread_mutex_unlock( &in->connin.ha5.head->connin.ha5.lock ) ;
		
		if ( ret ) {
			return -EIO ;
		}
	}
	return 0;
}

static int HA5_select_and_sendback(const BYTE * data, BYTE * resp, const size_t size, const struct parsedname *pn)
{
	struct connection_in * in = pn->selected_connection ;
	int left;
	char block_cmd ;

	if ( memcmp( pn->sn, in->connin.ha5.sn, 8 ) ) {
		// Need a formal change of device
		if ( HA5_select(pn) ) {
			return -EIO ;
		}
		block_cmd = 'W' ;
	} else {
		// Same device
		block_cmd = 'J' ;
	}
	
	for ( left=size ; left>0 ; left -= 32 ) {
		int ret ;
		size_t pass_start = size - left ;
		size_t pass_size = (left>32)?32:left ;
		
		pthread_mutex_lock( &in->connin.ha5.head->connin.ha5.lock ) ;
		ret = HA5_sendback_part( block_cmd, &data[pass_start], &resp[pass_start], pass_size, pn ) ;
		pthread_mutex_unlock( &in->connin.ha5.head->connin.ha5.lock ) ;
		block_cmd = 'W' ; // for next pass
		if ( ret ) {
			return -EIO ;
		}
	}
	return 0;
}

/********************************************************/
/* HA5_close  ** clean local resources before      	*/
/*                closing the serial port           	*/
/*							*/
/********************************************************/

static void HA5_close(struct connection_in *in)
{
	DirblobClear(&(in->connin.ha5.main));
	DirblobClear(&(in->connin.ha5.alarm));
	HA5_powerdown(in) ;
	if ( in->connin.ha5.head == in ) {
		my_pthread_mutex_destroy(&(in->connin.ha5.lock));
	}
	COM_close(in);
}

static void HA5_powerdown(struct connection_in * in)
{
	struct parsedname pn;

	FS_ParsedName(NULL, &pn);	// minimal parsename -- no destroy needed
	pn.selected_connection = in;

	COM_write((BYTE*)"P", 1, &pn) ;
	if ( in->file_descriptor > -1 ) {
		COM_slurp(in->file_descriptor) ;
	}
}
