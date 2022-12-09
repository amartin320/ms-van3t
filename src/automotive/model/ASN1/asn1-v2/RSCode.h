/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_RSCode_H_
#define	_RSCode_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"
#include "VcCode.h"
#include "ISO14823Code.h"
#include "AnyCatalogue.h"
#include "constr_CHOICE.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RSCode__code_PR {
	RSCode__code_PR_NOTHING,	/* No components present */
	RSCode__code_PR_viennaConvention,
	RSCode__code_PR_iso14823,
	RSCode__code_PR_itisCodes,
	RSCode__code_PR_anyCatalogue
	/* Extensions may appear below */
	
} RSCode__code_PR;

/* RSCode */
typedef struct RSCode {
	long	*layoutComponentId	/* OPTIONAL */;
	struct RSCode__code {
		RSCode__code_PR present;
		union RSCode__code_u {
			VcCode_t	 viennaConvention;
			ISO14823Code_t	 iso14823;
			long	 itisCodes;
			AnyCatalogue_t	 anyCatalogue;
			/*
			 * This type is extensible,
			 * possible extensions are below.
			 */
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} code;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RSCode_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RSCode;
extern asn_SEQUENCE_specifics_t asn_SPC_RSCode_specs_1;
extern asn_TYPE_member_t asn_MBR_RSCode_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _RSCode_H_ */
#include "asn_internal.h"
