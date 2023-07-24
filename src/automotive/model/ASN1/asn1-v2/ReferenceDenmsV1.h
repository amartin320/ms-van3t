/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DENM-PDU-Descriptions"
 * 	found in "DENM-PDU-Descriptions-1.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_ReferenceDenmsV1_H_
#define	_ReferenceDenmsV1_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ActionIDV1;

/* ReferenceDenmsV1 */
typedef struct ReferenceDenmsV1 {
	A_SEQUENCE_OF(struct ActionIDV1) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ReferenceDenmsV1_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ReferenceDenmsV1;
extern asn_SET_OF_specifics_t asn_SPC_ReferenceDenmsV1_specs_1;
extern asn_TYPE_member_t asn_MBR_ReferenceDenmsV1_1[1];
extern asn_per_constraints_t asn_PER_type_ReferenceDenmsV1_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ActionIDV1.h"

#endif	/* _ReferenceDenmsV1_H_ */
#include "asn_internal.h"