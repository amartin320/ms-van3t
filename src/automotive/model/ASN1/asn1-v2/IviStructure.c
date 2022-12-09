/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#include "IviStructure.h"

static int
memb_optional_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	/* Determine the number of elements */
	size = _A_CSEQUENCE_FROM_VOID(sptr)->count;
	
	if((size >= 1 && size <= 8)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_optional_constr_3 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_type_optional_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  1,  8 }	/* (SIZE(1..8,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_optional_constr_3 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_memb_optional_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  1,  8 }	/* (SIZE(1..8,...)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_optional_3[] = {
	{ ATF_POINTER, 0, 0,
		-1 /* Ambiguous tag (CHOICE?) */,
		0,
		&asn_DEF_IviContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_optional_tags_3[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_optional_specs_3 = {
	sizeof(struct IviStructure__optional),
	offsetof(struct IviStructure__optional, _asn_ctx),
	2,	/* XER encoding is XMLValueList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_optional_3 = {
	"optional",
	"optional",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_optional_tags_3,
	sizeof(asn_DEF_optional_tags_3)
		/sizeof(asn_DEF_optional_tags_3[0]) - 1, /* 1 */
	asn_DEF_optional_tags_3,	/* Same as above */
	sizeof(asn_DEF_optional_tags_3)
		/sizeof(asn_DEF_optional_tags_3[0]), /* 2 */
	{ &asn_OER_type_optional_constr_3, &asn_PER_type_optional_constr_3, SEQUENCE_OF_constraint },
	asn_MBR_optional_3,
	1,	/* Single element */
	&asn_SPC_optional_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_IviStructure_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct IviStructure, mandatory),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_IVIManagementContainer,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"mandatory"
		},
	{ ATF_POINTER, 1, offsetof(struct IviStructure, optional),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_optional_3,
		0,
		{ &asn_OER_memb_optional_constr_3, &asn_PER_memb_optional_constr_3,  memb_optional_constraint_1 },
		0, 0, /* No default value */
		"optional"
		},
};
static const int asn_MAP_IviStructure_oms_1[] = { 1 };
static const ber_tlv_tag_t asn_DEF_IviStructure_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_IviStructure_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* mandatory */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* optional */
};
asn_SEQUENCE_specifics_t asn_SPC_IviStructure_specs_1 = {
	sizeof(struct IviStructure),
	offsetof(struct IviStructure, _asn_ctx),
	asn_MAP_IviStructure_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_IviStructure_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_IviStructure = {
	"IviStructure",
	"IviStructure",
	&asn_OP_SEQUENCE,
	asn_DEF_IviStructure_tags_1,
	sizeof(asn_DEF_IviStructure_tags_1)
		/sizeof(asn_DEF_IviStructure_tags_1[0]), /* 1 */
	asn_DEF_IviStructure_tags_1,	/* Same as above */
	sizeof(asn_DEF_IviStructure_tags_1)
		/sizeof(asn_DEF_IviStructure_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_IviStructure_1,
	2,	/* Elements count */
	&asn_SPC_IviStructure_specs_1	/* Additional specs */
};

