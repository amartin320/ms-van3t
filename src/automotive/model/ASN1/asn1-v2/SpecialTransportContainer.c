/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CAM-PDU-Descriptions"
 * 	found in "../IVIM-ASN1-files/ETSI CAM v1.4.1.asn"
 * 	`asn1c -fcompound-names`
 */

#include "SpecialTransportContainer.h"

asn_TYPE_member_t asn_MBR_SpecialTransportContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct SpecialTransportContainer, specialTransportType),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SpecialTransportType,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"specialTransportType"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct SpecialTransportContainer, lightBarSirenInUse),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LightBarSirenInUse,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"lightBarSirenInUse"
		},
};
static const ber_tlv_tag_t asn_DEF_SpecialTransportContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_SpecialTransportContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* specialTransportType */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* lightBarSirenInUse */
};
asn_SEQUENCE_specifics_t asn_SPC_SpecialTransportContainer_specs_1 = {
	sizeof(struct SpecialTransportContainer),
	offsetof(struct SpecialTransportContainer, _asn_ctx),
	asn_MAP_SpecialTransportContainer_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_SpecialTransportContainer = {
	"SpecialTransportContainer",
	"SpecialTransportContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_SpecialTransportContainer_tags_1,
	sizeof(asn_DEF_SpecialTransportContainer_tags_1)
		/sizeof(asn_DEF_SpecialTransportContainer_tags_1[0]), /* 1 */
	asn_DEF_SpecialTransportContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_SpecialTransportContainer_tags_1)
		/sizeof(asn_DEF_SpecialTransportContainer_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_SpecialTransportContainer_1,
	2,	/* Elements count */
	&asn_SPC_SpecialTransportContainer_specs_1	/* Additional specs */
};

