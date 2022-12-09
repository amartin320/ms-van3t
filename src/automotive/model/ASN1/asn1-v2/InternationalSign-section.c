/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "GDD"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_14823_GDD.asn"
 * 	`asn1c -fcompound-names`
 */

#include "InternationalSign-section.h"

asn_TYPE_member_t asn_MBR_InternationalSign_section_1[] = {
	{ ATF_POINTER, 2, offsetof(struct InternationalSign_section, startingPointLength),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Distance,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"startingPointLength"
		},
	{ ATF_POINTER, 1, offsetof(struct InternationalSign_section, continuityLength),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Distance,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"continuityLength"
		},
};
static const int asn_MAP_InternationalSign_section_oms_1[] = { 0, 1 };
static const ber_tlv_tag_t asn_DEF_InternationalSign_section_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_InternationalSign_section_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* startingPointLength */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* continuityLength */
};
asn_SEQUENCE_specifics_t asn_SPC_InternationalSign_section_specs_1 = {
	sizeof(struct InternationalSign_section),
	offsetof(struct InternationalSign_section, _asn_ctx),
	asn_MAP_InternationalSign_section_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_InternationalSign_section_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_InternationalSign_section = {
	"InternationalSign-section",
	"InternationalSign-section",
	&asn_OP_SEQUENCE,
	asn_DEF_InternationalSign_section_tags_1,
	sizeof(asn_DEF_InternationalSign_section_tags_1)
		/sizeof(asn_DEF_InternationalSign_section_tags_1[0]), /* 1 */
	asn_DEF_InternationalSign_section_tags_1,	/* Same as above */
	sizeof(asn_DEF_InternationalSign_section_tags_1)
		/sizeof(asn_DEF_InternationalSign_section_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_InternationalSign_section_1,
	2,	/* Elements count */
	&asn_SPC_InternationalSign_section_specs_1	/* Additional specs */
};

