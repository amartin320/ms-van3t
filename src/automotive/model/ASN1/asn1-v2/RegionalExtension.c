/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC-REGION-noCircular"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19091_DSRC_REGION_noCircular.asn"
 * 	`asn1c -fcompound-names`
 */

#include "RegionalExtension.h"

static const asn_ioc_cell_t asn_IOS_Reg_LaneDataAttribute_1_rows[] = {
	
};
static const asn_ioc_set_t asn_IOS_Reg_LaneDataAttribute_1[] = {
	0, 0, asn_IOS_Reg_LaneDataAttribute_1_rows
};
static int
memb_regionId_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 255)) {
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
memb_regExtValue_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	if(1 /* No applicable constraints whatsoever */) {
		/* Nothing is here. See below */
	}
	
	return td->encoding_constraints.general_constraints(td, sptr, ctfailcb, app_key);
}

static asn_oer_constraints_t asn_OER_memb_regionId_constr_2 CC_NOTUSED = {
	{ 1, 1 }	/* (0..255) */,
	-1};
static asn_per_constraints_t asn_PER_memb_regionId_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 8,  8,  0,  255 }	/* (0..255) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_regExtValue_constr_3 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_memb_regExtValue_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_CHOICE_specifics_t asn_SPC_regExtValue_specs_3 = {
	sizeof(struct RegionalExtension_320P0__regExtValue),
	offsetof(struct RegionalExtension_320P0__regExtValue, _asn_ctx),
	offsetof(struct RegionalExtension_320P0__regExtValue, present),
	sizeof(((struct RegionalExtension_320P0__regExtValue *)0)->present),
	0,	/* No top level tags */
	0,	/* No tags in the map */
	0, 0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_regExtValue_3 = {
	"regExtValue",
	"regExtValue",
	&asn_OP_OPEN_TYPE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ 0, 0, OPEN_TYPE_constraint },
	0, 0,	/* No members */
	&asn_SPC_regExtValue_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_RegionalExtension_320P0_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RegionalExtension_320P0, regionId),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_RegionId,
		0,
		{ &asn_OER_memb_regionId_constr_2, &asn_PER_memb_regionId_constr_2,  memb_regionId_constraint_1 },
		0, 0, /* No default value */
		"regionId"
		},
	{ ATF_OPEN_TYPE | ATF_NOFLAGS, 0, offsetof(struct RegionalExtension_320P0, regExtValue),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_regExtValue_3,
		0,
		{ &asn_OER_memb_regExtValue_constr_3, &asn_PER_memb_regExtValue_constr_3,  memb_regExtValue_constraint_1 },
		0, 0, /* No default value */
		"regExtValue"
		},
};
static const ber_tlv_tag_t asn_DEF_RegionalExtension_320P0_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RegionalExtension_320P0_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* regionId */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* regExtValue */
};
asn_SEQUENCE_specifics_t asn_SPC_RegionalExtension_320P0_specs_1 = {
	sizeof(struct RegionalExtension_320P0),
	offsetof(struct RegionalExtension_320P0, _asn_ctx),
	asn_MAP_RegionalExtension_320P0_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RegionalExtension_320P0 = {
	"RegionalExtension",
	"RegionalExtension",
	&asn_OP_SEQUENCE,
	asn_DEF_RegionalExtension_320P0_tags_1,
	sizeof(asn_DEF_RegionalExtension_320P0_tags_1)
		/sizeof(asn_DEF_RegionalExtension_320P0_tags_1[0]), /* 1 */
	asn_DEF_RegionalExtension_320P0_tags_1,	/* Same as above */
	sizeof(asn_DEF_RegionalExtension_320P0_tags_1)
		/sizeof(asn_DEF_RegionalExtension_320P0_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_RegionalExtension_320P0_1,
	2,	/* Elements count */
	&asn_SPC_RegionalExtension_320P0_specs_1	/* Additional specs */
};

