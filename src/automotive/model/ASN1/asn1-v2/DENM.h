/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DENM-PDU-Descriptions"
 * 	found in "../IVIM-ASN1-files/ETSI DENM v1.3.1.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_DENM_H_
#define	_DENM_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ItsPduHeader.h"
#include "DecentralizedEnvironmentalNotificationMessage.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DENM */
typedef struct DENM {
	ItsPduHeader_t	 header;
	DecentralizedEnvironmentalNotificationMessage_t	 denm;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DENM_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DENM;

#ifdef __cplusplus
}
#endif

#endif	/* _DENM_H_ */
#include "asn_internal.h"
