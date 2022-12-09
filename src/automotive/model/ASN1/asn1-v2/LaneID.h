/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC-REGION-noCircular"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19091_DSRC_REGION_noCircular.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_LaneID_H_
#define	_LaneID_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* LaneID */
typedef long	 LaneID_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_LaneID_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_LaneID;
asn_struct_free_f LaneID_free;
asn_struct_print_f LaneID_print;
asn_constr_check_f LaneID_constraint;
ber_type_decoder_f LaneID_decode_ber;
der_type_encoder_f LaneID_encode_der;
xer_type_decoder_f LaneID_decode_xer;
xer_type_encoder_f LaneID_encode_xer;
oer_type_decoder_f LaneID_decode_oer;
oer_type_encoder_f LaneID_encode_oer;
per_type_decoder_f LaneID_decode_uper;
per_type_encoder_f LaneID_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _LaneID_H_ */
#include "asn_internal.h"
