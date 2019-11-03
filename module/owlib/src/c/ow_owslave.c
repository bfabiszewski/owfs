/*
    $Id: ow_owslave.c by baf $
    OWFS -- One-Wire filesystem
    OWHTTPD -- One-Wire Web Server
    Written 2003 Paul H Alfille
    email: palfille@earthlink.net
    Released under the GPL
    See the header file: ow.h for full attribution
    1wire/iButton system from Dallas Semiconductor
 */

/*  General Device File format:
    This device file corresponds to a specific 1wire/iButton chip type
    ( or a closely related family of chips )

    The connection to the larger program is through the "device" data structure,
    which must be declared in the acompanying header file.

    The device structure holds the
    family code,
    name,
    device type (chip, interface or pseudo)
    number of properties,
    list of property structures, called "filetype".

    Each filetype structure holds the
    name,
    estimated length (in bytes),
    aggregate structure pointer,
    data format,
    read function,
    write funtion,
    generic data pointer

    The aggregate structure, is present for properties that several members
    (e.g. pages of memory or entries in a temperature log. It holds:
    number of elements
    whether the members are lettered or numbered
    whether the elements are stored together and split, or separately and joined
 */

#include <config.h>
#include "owfs_config.h"
#include "ow_owslave.h"

/* ------- Prototypes ----------- */
READ_FUNCTION(FS_r_stype);
READ_FUNCTION(FS_r_temp);
READ_FUNCTION(FS_r_humi);
READ_FUNCTION(FS_l_temp);
READ_FUNCTION(FS_l_humi);

static enum e_visibility VISIBLE_SHT1X(const struct parsedname * pn);
static enum e_visibility VISIBLE_SHT2X(const struct parsedname * pn);

/* -------- Structures ---------- */

struct filetype OWSLAVE[] = {
    F_STANDARD,
    {"sensor_type", PROPERTY_LENGTH_UNSIGNED, NON_AGGREGATE, ft_unsigned, fc_stable, FS_r_stype, NO_WRITE_FUNCTION, VISIBLE, NO_FILETYPE_DATA,},
    {"temperature", PROPERTY_LENGTH_TEMP, NON_AGGREGATE, ft_temperature, fc_volatile, FS_r_temp, NO_WRITE_FUNCTION, VISIBLE, NO_FILETYPE_DATA,},
    {"humidity", PROPERTY_LENGTH_FLOAT, NON_AGGREGATE, ft_float, fc_volatile, FS_r_humi, NO_WRITE_FUNCTION, VISIBLE, NO_FILETYPE_DATA,},
    {"SHT1x", PROPERTY_LENGTH_SUBDIR, NON_AGGREGATE, ft_subdir, fc_subdir, NO_READ_FUNCTION, NO_WRITE_FUNCTION, VISIBLE_SHT1X, NO_FILETYPE_DATA,},
    {"SHT1x/temperature", PROPERTY_LENGTH_FLOAT, NON_AGGREGATE, ft_float, fc_link, FS_l_temp, NO_WRITE_FUNCTION, VISIBLE_SHT1X, NO_FILETYPE_DATA,},
    {"SHT1x/humidity", PROPERTY_LENGTH_FLOAT, NON_AGGREGATE, ft_float, fc_link, FS_l_humi, NO_WRITE_FUNCTION, VISIBLE_SHT1X, NO_FILETYPE_DATA,},
    {"SHT2x", PROPERTY_LENGTH_SUBDIR, NON_AGGREGATE, ft_subdir, fc_subdir, NO_READ_FUNCTION, NO_WRITE_FUNCTION, VISIBLE_SHT2X, NO_FILETYPE_DATA,},
    {"SHT2x/temperature", PROPERTY_LENGTH_FLOAT, NON_AGGREGATE, ft_float, fc_link, FS_l_temp, NO_WRITE_FUNCTION, VISIBLE_SHT2X, NO_FILETYPE_DATA,},
    {"SHT2x/humidity", PROPERTY_LENGTH_FLOAT, NON_AGGREGATE, ft_float, fc_link, FS_l_humi, NO_WRITE_FUNCTION, VISIBLE_SHT2X, NO_FILETYPE_DATA,}
};

DeviceEntryExtended(BF, OWSLAVE, DEV_temp, NO_GENERIC_READ, NO_GENERIC_WRITE);

#define _1W_READ_SCRATCHPAD       0xBE
#define _1W_CONVERT_T             0x44

/* ------- Functions ------------ */

static GOOD_OR_BAD OW_r_raw(BYTE * p, const struct parsedname *pn);
static unsigned crc8_add(unsigned acc, unsigned byte);
static unsigned char rev8bits(unsigned char v);

static GOOD_OR_BAD OW_r_raw(BYTE * p, const struct parsedname *pn)
{
    BYTE data[7];
    BYTE tcrc,hcrc;
    BYTE sensor_type;
    BYTE convert[] = { _1W_CONVERT_T, };
    BYTE r_scratchpad[] = { _1W_READ_SCRATCHPAD, };
    struct transaction_log t1[] = {
        TRXN_START,
        TRXN_WRITE1(convert),
        TRXN_END,
    };
    struct transaction_log t2[] = {
        TRXN_START,
        TRXN_DELAY(110),
        TRXN_WRITE1(r_scratchpad),
        TRXN_READ(data, 7),
        TRXN_END,
    };
    RETURN_BAD_IF_BAD(BUS_transaction(t1, pn));
    GOOD_OR_BAD ret = gbBAD;
    int i;
    // up to 400 ms for sht1x
    // up to 114 ms for sht2x
    for (i = 0; i < 5555; i++) {
        LEVEL_DEBUG("OWSLAVE %i pass",i);
        RETURN_BAD_IF_BAD(BUS_transaction(t2, pn));
        
        LEVEL_DEBUG("OWSLAVE RAW DATA: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
        //RETURN_BAD_IF_BAD(OW_r_scratchpad(data, pn));
        
        sensor_type = data[6];
        if (sensor_type == 2) {
            // sht2x - i2c
            hcrc = crc8_add(0x0, data[0]);
            hcrc = crc8_add(hcrc, data[1]);
            tcrc = crc8_add(0x0, data[3]);
            tcrc = crc8_add(tcrc, data[4]);
        } else {
            //sht1x sht7x
            hcrc = crc8_add(0x0, 0x05);
            hcrc = crc8_add(hcrc, data[0]);
            hcrc = crc8_add(hcrc, data[1]);
            hcrc = rev8bits(hcrc);
            tcrc = crc8_add(0x0, 0x03);
            tcrc = crc8_add(tcrc, data[3]);
            tcrc = crc8_add(tcrc, data[4]);
            tcrc = rev8bits(tcrc);
        }
        if (hcrc == data[2] && tcrc == data[5]) {
            ret = gbGOOD;
            break;
        }
        LEVEL_DEBUG("OWSLAVE CRC ERROR: hcrc: 0x%X rhcrc: 0x%X tcrc: 0x%X rtcrc: 0x%X", hcrc, data[2], tcrc, data[5]);
    }
    if (BAD(ret)) { return gbBAD; }
    memcpy(p, data, 7);
    return gbGOOD;
}
static ZERO_OR_ERROR FS_r_temp(struct one_wire_query *owq)
{
    struct parsedname * pn = PN(owq);
    BYTE data_raw[7];
    BYTE sensor_type,Tmsb,Tlsb;
    _FLOAT temp_true;
    _FLOAT t;
    
    RETURN_BAD_IF_BAD(OW_r_raw(data_raw, pn));
    Tmsb = data_raw[3];
    Tlsb = data_raw[4];
    sensor_type = data_raw[6];
    if (sensor_type == 2) {
        // sht2x i2c
        Tlsb = Tlsb & ~0x3;
        t = (_FLOAT)((Tmsb << 8) | Tlsb);
        temp_true = 175.72 * t / 65536 - 46.85;
    } else if (sensor_type == 1) {
        // sht1x sht7x
        t = (_FLOAT)((Tmsb << 8) | Tlsb);
        temp_true = t * 0.01 - 40.1;
    } else {
        return gbBAD;
    }
    
    LEVEL_DEBUG("OWLSAVE temperature: %.2f",temp_true);
    OWQ_F(owq) = temp_true;
    return gbGOOD;
}

/* get the temp from the scratchpad buffer after starting a conversion and waiting */
static ZERO_OR_ERROR FS_r_humi(struct one_wire_query *owq)
{
    struct parsedname * pn = PN(owq);
    BYTE data_raw[7];
    BYTE sensor_type,Tmsb,Tlsb,Hmsb,Hlsb;
    const float C1 = -2.0468;
    const float C2 = +0.0367;
    const float C3 = -0.0000015955;
    const float T1 = +0.01;
    const float T2 = +0.00008;
    float rh_lin,rh_true,temp_true;
    float rh, t;
    
    RETURN_BAD_IF_BAD(OW_r_raw(data_raw, pn));
    Hmsb = data_raw[0];
    Hlsb = data_raw[1];
    Tmsb = data_raw[3];
    Tlsb = data_raw[4];
    sensor_type = data_raw[6];
    
    if (sensor_type == 2) {
        // sht2x i2c
        Hlsb = Hlsb & ~0x3;
        rh = (_FLOAT)((Hmsb << 8) | Hlsb);
        rh_true = 125 * rh / 65536 - 6;
    } else if (sensor_type == 1) {
        // sht1x sht7x
        rh = (_FLOAT)((Hmsb << 8) | Hlsb);
        t = (_FLOAT)((Tmsb << 8) | Tlsb);
        temp_true = t * 0.01 - 40.1;
        rh_lin = C3 * rh * rh + C2 * rh + C1;
        rh_true = (temp_true - 25) * (T1 + T2 * rh) + rh_lin;
        if (rh_true > 100)
            rh_true = 100;
        if (rh_true < 0.1)
            rh_true = 0.1;
    } else {
        return gbBAD;
    }
    
    LEVEL_DEBUG("OWLSAVE humidity: %.2f",rh_true);
    OWQ_F(owq) = rh_true;
    return gbGOOD;
}

static ZERO_OR_ERROR FS_r_stype(struct one_wire_query *owq)
{
    struct parsedname * pn = PN(owq);
    BYTE data_raw[7];
    BYTE sensor_type;
    
    RETURN_BAD_IF_BAD(OW_r_raw(data_raw, pn));
    sensor_type = data_raw[6];
    
    LEVEL_DEBUG("OWLSAVE sensor_type: %i", sensor_type);
    OWQ_U(owq) = sensor_type;
    return gbGOOD;
}

static ZERO_OR_ERROR FS_l_humi(struct one_wire_query *owq)
{
    _FLOAT H = 0.;
    ZERO_OR_ERROR z_or_e = FS_r_sibling_F(&H, "humidity", owq) ;
    
    OWQ_F(owq) = H;
    return z_or_e;
}

static ZERO_OR_ERROR FS_l_temp(struct one_wire_query *owq)
{
    _FLOAT T = 0.;
    ZERO_OR_ERROR z_or_e = FS_r_sibling_F(&T, "temperature", owq) ;
    
    OWQ_F(owq) = T;
    return z_or_e;
}

static int VISIBLE_SENSOR(const struct parsedname * pn)
{
    int visibility_parameter = -1 ;
    
    LEVEL_DEBUG("Checking visibility of %s",SAFESTRING(pn->path)) ;
    if (BAD( GetVisibilityCache( &visibility_parameter, pn))) {
        struct one_wire_query * owq = OWQ_create_from_path(pn->path) ;
        if (owq != NULL) {
            UINT device_type ;
            if (FS_r_sibling_U( &device_type, "sensor_type", owq) == 0 ) {
                switch (device_type) {
                    case 1:
                        visibility_parameter = 1 ;
                        SetVisibilityCache(visibility_parameter, pn) ;
                        break ;
                    case 2:
                        visibility_parameter = 2 ;
                        SetVisibilityCache(visibility_parameter, pn) ;
                        break ;
                    default:
                        visibility_parameter = -1 ;
                }
            }
            OWQ_destroy(owq) ;
        }
    }
    return visibility_parameter ;
}

static enum e_visibility VISIBLE_SHT1X(const struct parsedname * pn)
{
    if (VISIBLE_SENSOR(pn) == 1) {
        return visible_now;
    } else {
        return visible_not_now;
    }
}

static enum e_visibility VISIBLE_SHT2X(const struct parsedname * pn)
{
    if (VISIBLE_SENSOR(pn) == 2) {
        return visible_now;
    } else {
        return visible_not_now;
    }
}

/* sht data crc */
static unsigned crc8_add(unsigned acc, unsigned byte)
{
    int i;
    acc ^= byte;
    for (i = 0; i < 8; i++) {
        if (acc & 0x80) {
            acc = (acc << 1) ^ 0x31;
        } else {
            acc <<= 1;
        }
    }
    return acc & 0xff;
}
static unsigned char rev8bits(unsigned char v)
{
    unsigned char r = v;
    int s = 7;
    for (v >>= 1; v; v >>= 1) {
        r <<= 1;
        r |= v & 1;
        s--;
    }
    r <<= s;
    return r;
}
