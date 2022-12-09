/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#include "RSCode.h"

static int
memb_itisCodes_constraint_3(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 65535)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_layoutComponentId_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 1 && value <= 4)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_memb_itisCodes_constr_6 CC_NOTUSED = {
	{ 2, 1 }	/* (0..65535) */,
	-1};
static asn_per_constraints_t asn_PER_memb_itisCodes_constr_6 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 16,  16,  0,  65535 }	/* (0..65535) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_code_constr_3 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_code_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  0,  3 }	/* (0..3,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_layoutComponentId_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_memb_layoutComponentId_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  1,  4 }	/* (1..4,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_code_3[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RSCode__code, choice.viennaConvention),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VcCode,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"viennaConvention"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RSCode__code, choice.iso14823),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ISO14823Code,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"iso14823"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RSCode__code, choice.itisCodes),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_itisCodes_constr_6, &asn_PER_memb_itisCodes_constr_6,  memb_itisCodes_constraint_3 },
		0, 0, /* No default value */
		"itisCodes"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RSCode__code, choice.anyCatalogue),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_AnyCatalogue,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"anyCatalogue"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_code_tag2el_3[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* viennaConvention */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* iso14823 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* itisCodes */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* anyCatalogue */
};
static asn_CHOICE_specifics_t asn_SPC_code_specs_3 = {
	sizeof(struct RSCode__code),
	offsetof(struct RSCode__code, _asn_ctx),
	offsetof(struct RSCode__code, present),
	sizeof(((struct RSCode__code *)0)->present),
	asn_MAP_code_tag2el_3,
	4,	/* Count of tags in the map */
	0, 0,
	4	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_code_3 = {
	"code",
	"code",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_code_constr_3, &asn_PER_type_code_constr_3, CHOICE_constraint },
	asn_MBR_code_3,
	4,	/* Elements count */
	&asn_SPC_code_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_RSCode_1[] = {
	{ ATF_POINTER, 1, offsetof(struct RSCode, layoutComponentId),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_layoutComponentId_constr_2, &asn_PER_memb_layoutComponentId_constr_2,  memb_layoutComponentId_constraint_1 },
		0, 0, /* No default value */
		"layoutComponentId"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RSCode, code),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_code_3,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"code"
		},
};
static const int asn_MAP_RSCode_oms_1[] = { 0 };
static const ber_tlv_tag_t asn_DEF_RSCode_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RSCode_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* layoutComponentId */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* code */
};
asn_SEQUENCE_specifics_t asn_SPC_RSCode_specs_1 = {
	sizeof(struct RSCode),
	offsetof(struct RSCode, _asn_ctx),
	asn_MAP_RSCode_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_RSCode_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RSCode = {
	"RSCode",
	"RSCode",
	&asn_OP_SEQUENCE,
	asn_DEF_RSCode_tags_1,
	sizeof(asn_DEF_RSCode_tags_1)
		/sizeof(asn_DEF_RSCode_tags_1[0]), /* 1 */
	asn_DEF_RSCode_tags_1,	/* Same as above */
	sizeof(asn_DEF_RSCode_tags_1)
		/sizeof(asn_DEF_RSCode_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_RSCode_1,
	2,	/* Elements count */
	&asn_SPC_RSCode_specs_1	/* Additional specs */
};

