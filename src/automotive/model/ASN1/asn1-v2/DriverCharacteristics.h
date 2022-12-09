/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_DriverCharacteristics_H_
#define	_DriverCharacteristics_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DriverCharacteristics {
	DriverCharacteristics_unexperiencedDrivers	= 0,
	DriverCharacteristics_experiencedDrivers	= 1,
	DriverCharacteristics_rfu1	= 2,
	DriverCharacteristics_rfu2	= 3
} e_DriverCharacteristics;

/* DriverCharacteristics */
typedef long	 DriverCharacteristics_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_DriverCharacteristics_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_DriverCharacteristics;
asn_struct_free_f DriverCharacteristics_free;
asn_struct_print_f DriverCharacteristics_print;
asn_constr_check_f DriverCharacteristics_constraint;
ber_type_decoder_f DriverCharacteristics_decode_ber;
der_type_encoder_f DriverCharacteristics_encode_der;
xer_type_decoder_f DriverCharacteristics_decode_xer;
xer_type_encoder_f DriverCharacteristics_encode_xer;
oer_type_decoder_f DriverCharacteristics_decode_oer;
oer_type_encoder_f DriverCharacteristics_encode_oer;
per_type_decoder_f DriverCharacteristics_decode_uper;
per_type_encoder_f DriverCharacteristics_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _DriverCharacteristics_H_ */
#include "asn_internal.h"
