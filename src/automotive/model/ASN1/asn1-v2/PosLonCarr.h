/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "../IVIM-ASN1-files/ITS-Container.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_PosLonCarr_H_
#define	_PosLonCarr_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PosLonCarr {
	PosLonCarr_oneCentimeter	= 1,
	PosLonCarr_unavailable	= 127
} e_PosLonCarr;

/* PosLonCarr */
typedef long	 PosLonCarr_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_PosLonCarr_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_PosLonCarr;
asn_struct_free_f PosLonCarr_free;
asn_struct_print_f PosLonCarr_print;
asn_constr_check_f PosLonCarr_constraint;
ber_type_decoder_f PosLonCarr_decode_ber;
der_type_encoder_f PosLonCarr_encode_der;
xer_type_decoder_f PosLonCarr_decode_xer;
xer_type_encoder_f PosLonCarr_encode_xer;
oer_type_decoder_f PosLonCarr_decode_oer;
oer_type_encoder_f PosLonCarr_encode_oer;
per_type_decoder_f PosLonCarr_decode_uper;
per_type_encoder_f PosLonCarr_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _PosLonCarr_H_ */
#include "asn_internal.h"
