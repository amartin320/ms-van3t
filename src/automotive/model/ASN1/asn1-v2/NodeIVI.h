/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC-REGION-noCircular"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19091_DSRC_REGION_noCircular.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_NodeIVI_H_
#define	_NodeIVI_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"
#include "LaneID.h"
#include "LaneConnectionID.h"
#include "IntersectionID.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* NodeIVI */
typedef struct NodeIVI {
	long	 id;
	LaneID_t	*lane	/* OPTIONAL */;
	LaneConnectionID_t	*connectionID	/* OPTIONAL */;
	IntersectionID_t	*intersectionID	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NodeIVI_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NodeIVI;
extern asn_SEQUENCE_specifics_t asn_SPC_NodeIVI_specs_1;
extern asn_TYPE_member_t asn_MBR_NodeIVI_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _NodeIVI_H_ */
#include "asn_internal.h"
