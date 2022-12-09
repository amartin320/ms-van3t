/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_14906_EfcDsrcApplication.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_ReceiptAuthenticator_H_
#define	_ReceiptAuthenticator_H_


#include "asn_application.h"

/* Including external dependencies */
#include "OCTET_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ReceiptAuthenticator */
typedef OCTET_STRING_t	 ReceiptAuthenticator_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ReceiptAuthenticator;
asn_struct_free_f ReceiptAuthenticator_free;
asn_struct_print_f ReceiptAuthenticator_print;
asn_constr_check_f ReceiptAuthenticator_constraint;
ber_type_decoder_f ReceiptAuthenticator_decode_ber;
der_type_encoder_f ReceiptAuthenticator_encode_der;
xer_type_decoder_f ReceiptAuthenticator_decode_xer;
xer_type_encoder_f ReceiptAuthenticator_encode_xer;
oer_type_decoder_f ReceiptAuthenticator_decode_oer;
oer_type_encoder_f ReceiptAuthenticator_encode_oer;
per_type_decoder_f ReceiptAuthenticator_decode_uper;
per_type_encoder_f ReceiptAuthenticator_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _ReceiptAuthenticator_H_ */
#include "asn_internal.h"
