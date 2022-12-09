/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_DDD_IO_LIST_H_
#define	_DDD_IO_LIST_H_


#include "asn_application.h"

/* Including external dependencies */
#include "asn_SEQUENCE_OF.h"
#include "constr_SEQUENCE_OF.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct DDD_IO;

/* DDD-IO-LIST */
typedef struct DDD_IO_LIST {
	A_SEQUENCE_OF(struct DDD_IO) list;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DDD_IO_LIST_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DDD_IO_LIST;
extern asn_SET_OF_specifics_t asn_SPC_DDD_IO_LIST_specs_1;
extern asn_TYPE_member_t asn_MBR_DDD_IO_LIST_1[1];
extern asn_per_constraints_t asn_PER_type_DDD_IO_LIST_constr_1;

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "DDD-IO.h"

#endif	/* _DDD_IO_LIST_H_ */
#include "asn_internal.h"
