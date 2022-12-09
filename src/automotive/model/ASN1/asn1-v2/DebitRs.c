/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_14906_EfcDsrcApplication.asn"
 * 	`asn1c -fcompound-names`
 */

#include "DebitRs.h"

static asn_TYPE_member_t asn_MBR_DebitRs_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct DebitRs, debitResult),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ResultFin,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"debitResult"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct DebitRs, debitAuthenticator),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"debitAuthenticator"
		},
};
static const ber_tlv_tag_t asn_DEF_DebitRs_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_DebitRs_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* debitResult */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* debitAuthenticator */
};
static asn_SEQUENCE_specifics_t asn_SPC_DebitRs_specs_1 = {
	sizeof(struct DebitRs),
	offsetof(struct DebitRs, _asn_ctx),
	asn_MAP_DebitRs_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_DebitRs = {
	"DebitRs",
	"DebitRs",
	&asn_OP_SEQUENCE,
	asn_DEF_DebitRs_tags_1,
	sizeof(asn_DEF_DebitRs_tags_1)
		/sizeof(asn_DEF_DebitRs_tags_1[0]), /* 1 */
	asn_DEF_DebitRs_tags_1,	/* Same as above */
	sizeof(asn_DEF_DebitRs_tags_1)
		/sizeof(asn_DEF_DebitRs_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_DebitRs_1,
	2,	/* Elements count */
	&asn_SPC_DebitRs_specs_1	/* Additional specs */
};

