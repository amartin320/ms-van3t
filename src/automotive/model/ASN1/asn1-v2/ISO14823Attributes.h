/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_ISO14823Attributes_H_
#define	_ISO14823Attributes_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct ISO14823Attribute;

/* ISO14823Attributes */
typedef struct ISO14823Attributes {
	A_SEQUENCE_OF(struct ISO14823Attribute) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ISO14823Attributes_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ISO14823Attributes;
extern asn_SET_OF_specifics_t asn_SPC_ISO14823Attributes_specs_1;
extern asn_TYPE_member_t asn_MBR_ISO14823Attributes_1[1];
extern asn_per_constraints_t asn_PER_type_ISO14823Attributes_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "ISO14823Attribute.h"

#endif	/* _ISO14823Attributes_H_ */
#include "asn_internal.h"
