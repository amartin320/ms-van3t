/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/home/carlosrisma/IVIM ASN1 files/asn1_IS_ISO_TS_14906_EfcDsrcApplication.asn"
 * 	`asn1c -fincludes-quoted`
 */

#ifndef	_ReceiptData_H_
#define	_ReceiptData_H_


#include "asn_application.h"

/* Including external dependencies */
#include "DateAndTime.h"
#include "Provider.h"
#include "Int2.h"
#include "SessionLocation.h"
#include "Int1.h"
#include "ResultOp.h"
#include "PaymentFee.h"
#include "OCTET_STRING.h"
#include "NativeInteger.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ReceiptData */
typedef struct ReceiptData {
	DateAndTime_t	 sessionTime;
	Provider_t	 sessionServiceProvider;
	Int2_t	 locationOfStation;
	SessionLocation_t	 sessionLocation;
	Int1_t	 sessionType;
	ResultOp_t	 sessionResult;
	Int1_t	 sessionTariffClass;
	Int1_t	 sessionClaimedClass;
	PaymentFee_t	 sessionFee;
	Provider_t	 sessionContractProvider;
	OCTET_STRING_t	 sessionTypeOfContract;
	long	 sessionContextVersion;
	OCTET_STRING_t	 receiptDataAuthenticator;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ReceiptData_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ReceiptData;
extern asn_SEQUENCE_specifics_t asn_SPC_ReceiptData_specs_1;
extern asn_TYPE_member_t asn_MBR_ReceiptData_1[13];

#ifdef __cplusplus
}
#endif

#endif	/* _ReceiptData_H_ */
#include "asn_internal.h"