/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "GDD"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_14823_GDD.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_Code_Units_H_
#define	_Code_Units_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum Code_Units {
	Code_Units_kmperh	= 0,
	Code_Units_milesperh	= 1,
	Code_Units_kilometre	= 2,
	Code_Units_metre	= 3,
	Code_Units_decimetre	= 4,
	Code_Units_centimetre	= 5,
	Code_Units_mile	= 6,
	Code_Units_yard	= 7,
	Code_Units_foot	= 8,
	Code_Units_minutesOfTime	= 9,
	Code_Units_tonnes	= 10,
	Code_Units_hundredkg	= 11,
	Code_Units_pound	= 12,
	Code_Units_rateOfIncline	= 13,
	Code_Units_durationinminutes	= 14
} e_Code_Units;

/* Code-Units */
typedef long	 Code_Units_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_Code_Units_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_Code_Units;
asn_struct_free_f Code_Units_free;
asn_struct_print_f Code_Units_print;
asn_constr_check_f Code_Units_constraint;
ber_type_decoder_f Code_Units_decode_ber;
der_type_encoder_f Code_Units_encode_der;
xer_type_decoder_f Code_Units_decode_xer;
xer_type_encoder_f Code_Units_encode_xer;
oer_type_decoder_f Code_Units_decode_oer;
oer_type_encoder_f Code_Units_encode_oer;
per_type_decoder_f Code_Units_decode_uper;
per_type_encoder_f Code_Units_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _Code_Units_H_ */
#include "asn_internal.h"
