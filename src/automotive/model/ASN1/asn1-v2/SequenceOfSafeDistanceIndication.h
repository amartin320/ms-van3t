/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "ETSI-ITS-CDD.asn"
 */

#ifndef	_SequenceOfSafeDistanceIndication_H_
#define	_SequenceOfSafeDistanceIndication_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct SafeDistanceIndication;

/* SequenceOfSafeDistanceIndication */
typedef struct SequenceOfSafeDistanceIndication {
	A_SEQUENCE_OF(struct SafeDistanceIndication) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SequenceOfSafeDistanceIndication_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SequenceOfSafeDistanceIndication;
extern asn_SET_OF_specifics_t asn_SPC_SequenceOfSafeDistanceIndication_specs_1;
extern asn_TYPE_member_t asn_MBR_SequenceOfSafeDistanceIndication_1[1];
extern asn_per_constraints_t asn_PER_type_SequenceOfSafeDistanceIndication_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SafeDistanceIndication.h"

#endif	/* _SequenceOfSafeDistanceIndication_H_ */
#include "asn_internal.h"