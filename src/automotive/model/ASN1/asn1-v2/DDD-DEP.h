/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_DDD_DEP_H_
#define	_DDD_DEP_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DDD_DEP {
	DDD_DEP_none	= 0,
	DDD_DEP_importantArea	= 1,
	DDD_DEP_principalArea	= 2,
	DDD_DEP_generalArea	= 3,
	DDD_DEP_wellKnownPoint	= 4,
	DDD_DEP_country	= 5,
	DDD_DEP_city	= 6,
	DDD_DEP_street	= 7,
	DDD_DEP_industrialArea	= 8,
	DDD_DEP_historicArea	= 9,
	DDD_DEP_touristicArea	= 10,
	DDD_DEP_culturalArea	= 11,
	DDD_DEP_touristicRoute	= 12,
	DDD_DEP_recommendedRoute	= 13,
	DDD_DEP_touristicAttraction	= 14,
	DDD_DEP_geographicArea	= 15
} e_DDD_DEP;

/* DDD-DEP */
typedef long	 DDD_DEP_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_DDD_DEP_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_DDD_DEP;
asn_struct_free_f DDD_DEP_free;
asn_struct_print_f DDD_DEP_print;
asn_constr_check_f DDD_DEP_constraint;
ber_type_decoder_f DDD_DEP_decode_ber;
der_type_encoder_f DDD_DEP_encode_der;
xer_type_decoder_f DDD_DEP_decode_xer;
xer_type_encoder_f DDD_DEP_encode_xer;
oer_type_decoder_f DDD_DEP_decode_oer;
oer_type_encoder_f DDD_DEP_encode_oer;
per_type_decoder_f DDD_DEP_decode_uper;
per_type_encoder_f DDD_DEP_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _DDD_DEP_H_ */
#include "asn_internal.h"
