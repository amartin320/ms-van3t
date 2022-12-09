/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CAM-PDU-Descriptions"
 * 	found in "../IVIM-ASN1-files/ETSI CAM v1.4.1.asn"
 * 	`asn1c -fcompound-names`
 */

#include "CamParameters.h"

asn_TYPE_member_t asn_MBR_CamParameters_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CamParameters, basicContainer),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BasicContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"basicContainer"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CamParameters, highFrequencyContainer),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_HighFrequencyContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"highFrequencyContainer"
		},
	{ ATF_POINTER, 2, offsetof(struct CamParameters, lowFrequencyContainer),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_LowFrequencyContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"lowFrequencyContainer"
		},
	{ ATF_POINTER, 1, offsetof(struct CamParameters, specialVehicleContainer),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SpecialVehicleContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"specialVehicleContainer"
		},
};
static const int asn_MAP_CamParameters_oms_1[] = { 2, 3 };
static const ber_tlv_tag_t asn_DEF_CamParameters_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CamParameters_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* basicContainer */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* highFrequencyContainer */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* lowFrequencyContainer */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* specialVehicleContainer */
};
asn_SEQUENCE_specifics_t asn_SPC_CamParameters_specs_1 = {
	sizeof(struct CamParameters),
	offsetof(struct CamParameters, _asn_ctx),
	asn_MAP_CamParameters_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_CamParameters_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CamParameters = {
	"CamParameters",
	"CamParameters",
	&asn_OP_SEQUENCE,
	asn_DEF_CamParameters_tags_1,
	sizeof(asn_DEF_CamParameters_tags_1)
		/sizeof(asn_DEF_CamParameters_tags_1[0]), /* 1 */
	asn_DEF_CamParameters_tags_1,	/* Same as above */
	sizeof(asn_DEF_CamParameters_tags_1)
		/sizeof(asn_DEF_CamParameters_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_CamParameters_1,
	4,	/* Elements count */
	&asn_SPC_CamParameters_specs_1	/* Additional specs */
};

