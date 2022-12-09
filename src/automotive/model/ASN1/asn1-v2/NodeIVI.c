/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC-REGION-noCircular"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19091_DSRC_REGION_noCircular.asn"
 * 	`asn1c -fcompound-names`
 */

#include "NodeIVI.h"

asn_TYPE_member_t asn_MBR_NodeIVI_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct NodeIVI, id),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"id"
		},
	{ ATF_POINTER, 3, offsetof(struct NodeIVI, lane),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LaneID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"lane"
		},
	{ ATF_POINTER, 2, offsetof(struct NodeIVI, connectionID),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LaneConnectionID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"connectionID"
		},
	{ ATF_POINTER, 1, offsetof(struct NodeIVI, intersectionID),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_IntersectionID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"intersectionID"
		},
};
static const int asn_MAP_NodeIVI_oms_1[] = { 1, 2, 3 };
static const ber_tlv_tag_t asn_DEF_NodeIVI_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_NodeIVI_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* id */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* lane */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* connectionID */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* intersectionID */
};
asn_SEQUENCE_specifics_t asn_SPC_NodeIVI_specs_1 = {
	sizeof(struct NodeIVI),
	offsetof(struct NodeIVI, _asn_ctx),
	asn_MAP_NodeIVI_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_NodeIVI_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_NodeIVI = {
	"NodeIVI",
	"NodeIVI",
	&asn_OP_SEQUENCE,
	asn_DEF_NodeIVI_tags_1,
	sizeof(asn_DEF_NodeIVI_tags_1)
		/sizeof(asn_DEF_NodeIVI_tags_1[0]), /* 1 */
	asn_DEF_NodeIVI_tags_1,	/* Same as above */
	sizeof(asn_DEF_NodeIVI_tags_1)
		/sizeof(asn_DEF_NodeIVI_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_NodeIVI_1,
	4,	/* Elements count */
	&asn_SPC_NodeIVI_specs_1	/* Additional specs */
};

