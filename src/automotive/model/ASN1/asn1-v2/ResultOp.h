/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_14906_EfcDsrcApplication.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_ResultOp_H_
#define	_ResultOp_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ResultOp {
	ResultOp_correctTransaction	= 0,
	ResultOp_obeStatusNotAccepted	= 1,
	ResultOp_equipmentStatusNotAccepted	= 2,
	ResultOp_contractNotInWhiteList	= 3,
	ResultOp_contractIdentifierInBlackList	= 4,
	ResultOp_contractIdentifierNotCorrect	= 5,
	ResultOp_expiredContract	= 6,
	ResultOp_contractRestrictionsNotFulfilled	= 7,
	ResultOp_claimedVehicleCharacteristicsNotValid	= 8,
	ResultOp_vehicleClassAuthenticationFailed	= 9,
	ResultOp_entryVehicleClassDifferentFromExitVehicleClass	= 10,
	ResultOp_entryReceiptMissing	= 11,
	ResultOp_entryReceiptNotValid	= 12,
	ResultOp_entryTollStationNotValid	= 13,
	ResultOp_equipmentNotCertified	= 14,
	ResultOp_timeDifference	= 15,
	ResultOp_accessCredentialsNotAccepted	= 16,
	ResultOp_contractAuthenticatorNotAccepted	= 17,
	ResultOp_receiptAuthenticatorNotAccepted	= 18,
	ResultOp_claimedVehicleCharacteristicsMissing	= 19,
	ResultOp_paymentMeansNotAccepted	= 20,
	ResultOp_paymentAuthenticatorNotAccepted	= 21,
	ResultOp_paymentMeansInBlackList	= 22,
	ResultOp_paymentMeansNotCorrect	= 23,
	ResultOp_expiredPaymentMeans	= 24,
	ResultOp_paymentMeansRestrictionsNotFulfilled	= 25
} e_ResultOp;

/* ResultOp */
typedef long	 ResultOp_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_ResultOp_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_ResultOp;
asn_struct_free_f ResultOp_free;
asn_struct_print_f ResultOp_print;
asn_constr_check_f ResultOp_constraint;
ber_type_decoder_f ResultOp_decode_ber;
der_type_encoder_f ResultOp_encode_der;
xer_type_decoder_f ResultOp_decode_xer;
xer_type_encoder_f ResultOp_encode_xer;
oer_type_decoder_f ResultOp_decode_oer;
oer_type_encoder_f ResultOp_encode_oer;
per_type_decoder_f ResultOp_decode_uper;
per_type_encoder_f ResultOp_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _ResultOp_H_ */
#include "asn_internal.h"
