/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_GlcPart_H_
#define	_GlcPart_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Zid.h"
#include "LanePosition.h"
#include "NativeInteger.h"
#include "HeadingValue.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct Zone;

/* GlcPart */
typedef struct GlcPart {
	Zid_t	 zoneId;
	LanePosition_t	*laneNumber	/* OPTIONAL */;
	long	*zoneExtension	/* OPTIONAL */;
	HeadingValue_t	*zoneHeading	/* OPTIONAL */;
	struct Zone	*zone	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} GlcPart_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_GlcPart;
extern asn_SEQUENCE_specifics_t asn_SPC_GlcPart_specs_1;
extern asn_TYPE_member_t asn_MBR_GlcPart_1[5];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "Zone.h"

#endif	/* _GlcPart_H_ */
#include "asn_internal.h"
