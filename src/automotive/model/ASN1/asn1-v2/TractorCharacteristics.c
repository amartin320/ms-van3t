/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#include "TractorCharacteristics.h"

static int
memb_equalTo_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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
	
	if((size >= 1 && size <= 4)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_notEqualTo_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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
	
	if((size >= 1 && size <= 4)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_ranges_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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
	
	if((size >= 1 && size <= 4)) {
		/* Perform validation of the inner elements */
		return SEQUENCE_OF_constraint(td, sptr, ctfailcb, app_key);
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_type_equalTo_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_type_equalTo_constr_2 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  1,  4 }	/* (SIZE(1..4,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_notEqualTo_constr_4 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_type_notEqualTo_constr_4 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  1,  4 }	/* (SIZE(1..4,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_ranges_constr_6 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_type_ranges_constr_6 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  1,  4 }	/* (SIZE(1..4,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_equalTo_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_memb_equalTo_constr_2 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  1,  4 }	/* (SIZE(1..4,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_notEqualTo_constr_4 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_memb_notEqualTo_constr_4 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  1,  4 }	/* (SIZE(1..4,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_ranges_constr_6 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_memb_ranges_constr_6 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  1,  4 }	/* (SIZE(1..4,...)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_equalTo_2[] = {
	{ ATF_POINTER, 0, 0,
		-1 /* Ambiguous tag (CHOICE?) */,
		0,
		&asn_DEF_VehicleCharacteristicsFixValues,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_equalTo_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_equalTo_specs_2 = {
	sizeof(struct TractorCharacteristics__equalTo),
	offsetof(struct TractorCharacteristics__equalTo, _asn_ctx),
	2,	/* XER encoding is XMLValueList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_equalTo_2 = {
	"equalTo",
	"equalTo",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_equalTo_tags_2,
	sizeof(asn_DEF_equalTo_tags_2)
		/sizeof(asn_DEF_equalTo_tags_2[0]) - 1, /* 1 */
	asn_DEF_equalTo_tags_2,	/* Same as above */
	sizeof(asn_DEF_equalTo_tags_2)
		/sizeof(asn_DEF_equalTo_tags_2[0]), /* 2 */
	{ &asn_OER_type_equalTo_constr_2, &asn_PER_type_equalTo_constr_2, SEQUENCE_OF_constraint },
	asn_MBR_equalTo_2,
	1,	/* Single element */
	&asn_SPC_equalTo_specs_2	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_notEqualTo_4[] = {
	{ ATF_POINTER, 0, 0,
		-1 /* Ambiguous tag (CHOICE?) */,
		0,
		&asn_DEF_VehicleCharacteristicsFixValues,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_notEqualTo_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_notEqualTo_specs_4 = {
	sizeof(struct TractorCharacteristics__notEqualTo),
	offsetof(struct TractorCharacteristics__notEqualTo, _asn_ctx),
	2,	/* XER encoding is XMLValueList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_notEqualTo_4 = {
	"notEqualTo",
	"notEqualTo",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_notEqualTo_tags_4,
	sizeof(asn_DEF_notEqualTo_tags_4)
		/sizeof(asn_DEF_notEqualTo_tags_4[0]) - 1, /* 1 */
	asn_DEF_notEqualTo_tags_4,	/* Same as above */
	sizeof(asn_DEF_notEqualTo_tags_4)
		/sizeof(asn_DEF_notEqualTo_tags_4[0]), /* 2 */
	{ &asn_OER_type_notEqualTo_constr_4, &asn_PER_type_notEqualTo_constr_4, SEQUENCE_OF_constraint },
	asn_MBR_notEqualTo_4,
	1,	/* Single element */
	&asn_SPC_notEqualTo_specs_4	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_ranges_6[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_VehicleCharacteristicsRanges,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_ranges_tags_6[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_ranges_specs_6 = {
	sizeof(struct TractorCharacteristics__ranges),
	offsetof(struct TractorCharacteristics__ranges, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ranges_6 = {
	"ranges",
	"ranges",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_ranges_tags_6,
	sizeof(asn_DEF_ranges_tags_6)
		/sizeof(asn_DEF_ranges_tags_6[0]) - 1, /* 1 */
	asn_DEF_ranges_tags_6,	/* Same as above */
	sizeof(asn_DEF_ranges_tags_6)
		/sizeof(asn_DEF_ranges_tags_6[0]), /* 2 */
	{ &asn_OER_type_ranges_constr_6, &asn_PER_type_ranges_constr_6, SEQUENCE_OF_constraint },
	asn_MBR_ranges_6,
	1,	/* Single element */
	&asn_SPC_ranges_specs_6	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_TractorCharacteristics_1[] = {
	{ ATF_POINTER, 3, offsetof(struct TractorCharacteristics, equalTo),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_equalTo_2,
		0,
		{ &asn_OER_memb_equalTo_constr_2, &asn_PER_memb_equalTo_constr_2,  memb_equalTo_constraint_1 },
		0, 0, /* No default value */
		"equalTo"
		},
	{ ATF_POINTER, 2, offsetof(struct TractorCharacteristics, notEqualTo),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_notEqualTo_4,
		0,
		{ &asn_OER_memb_notEqualTo_constr_4, &asn_PER_memb_notEqualTo_constr_4,  memb_notEqualTo_constraint_1 },
		0, 0, /* No default value */
		"notEqualTo"
		},
	{ ATF_POINTER, 1, offsetof(struct TractorCharacteristics, ranges),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		0,
		&asn_DEF_ranges_6,
		0,
		{ &asn_OER_memb_ranges_constr_6, &asn_PER_memb_ranges_constr_6,  memb_ranges_constraint_1 },
		0, 0, /* No default value */
		"ranges"
		},
};
static const int asn_MAP_TractorCharacteristics_oms_1[] = { 0, 1, 2 };
static const ber_tlv_tag_t asn_DEF_TractorCharacteristics_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_TractorCharacteristics_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* equalTo */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* notEqualTo */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* ranges */
};
asn_SEQUENCE_specifics_t asn_SPC_TractorCharacteristics_specs_1 = {
	sizeof(struct TractorCharacteristics),
	offsetof(struct TractorCharacteristics, _asn_ctx),
	asn_MAP_TractorCharacteristics_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_TractorCharacteristics_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_TractorCharacteristics = {
	"TractorCharacteristics",
	"TractorCharacteristics",
	&asn_OP_SEQUENCE,
	asn_DEF_TractorCharacteristics_tags_1,
	sizeof(asn_DEF_TractorCharacteristics_tags_1)
		/sizeof(asn_DEF_TractorCharacteristics_tags_1[0]), /* 1 */
	asn_DEF_TractorCharacteristics_tags_1,	/* Same as above */
	sizeof(asn_DEF_TractorCharacteristics_tags_1)
		/sizeof(asn_DEF_TractorCharacteristics_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_TractorCharacteristics_1,
	3,	/* Elements count */
	&asn_SPC_TractorCharacteristics_specs_1	/* Additional specs */
};

