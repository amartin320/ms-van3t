/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#include "PolygonalLine.h"

static int
memb_deltaPositions_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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
	
	if((size >= 1 && size <= 32)) {
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
memb_deltaPositionsWithAltitude_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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
	
	if((size >= 1 && size <= 32)) {
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
memb_absolutePositions_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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

static int
memb_absolutePositionsWithAltitude_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
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

static asn_oer_constraints_t asn_OER_type_deltaPositions_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_type_deltaPositions_constr_2 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  5,  5,  1,  32 }	/* (SIZE(1..32,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_deltaPositionsWithAltitude_constr_4 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_type_deltaPositionsWithAltitude_constr_4 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  5,  5,  1,  32 }	/* (SIZE(1..32,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_absolutePositions_constr_6 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_type_absolutePositions_constr_6 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  1,  8 }	/* (SIZE(1..8,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_absolutePositionsWithAltitude_constr_8 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_type_absolutePositionsWithAltitude_constr_8 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  1,  8 }	/* (SIZE(1..8,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_deltaPositions_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_memb_deltaPositions_constr_2 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  5,  5,  1,  32 }	/* (SIZE(1..32,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_deltaPositionsWithAltitude_constr_4 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_memb_deltaPositionsWithAltitude_constr_4 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  5,  5,  1,  32 }	/* (SIZE(1..32,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_absolutePositions_constr_6 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_memb_absolutePositions_constr_6 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  1,  8 }	/* (SIZE(1..8,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_absolutePositionsWithAltitude_constr_8 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(0..MAX)) */};
static asn_per_constraints_t asn_PER_memb_absolutePositionsWithAltitude_constr_8 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  1,  8 }	/* (SIZE(1..8,...)) */,
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_PolygonalLine_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_PolygonalLine_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  2,  2,  0,  3 }	/* (0..3,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_deltaPositions_2[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_DeltaPosition,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_deltaPositions_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_deltaPositions_specs_2 = {
	sizeof(struct PolygonalLine__deltaPositions),
	offsetof(struct PolygonalLine__deltaPositions, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_deltaPositions_2 = {
	"deltaPositions",
	"deltaPositions",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_deltaPositions_tags_2,
	sizeof(asn_DEF_deltaPositions_tags_2)
		/sizeof(asn_DEF_deltaPositions_tags_2[0]) - 1, /* 1 */
	asn_DEF_deltaPositions_tags_2,	/* Same as above */
	sizeof(asn_DEF_deltaPositions_tags_2)
		/sizeof(asn_DEF_deltaPositions_tags_2[0]), /* 2 */
	{ &asn_OER_type_deltaPositions_constr_2, &asn_PER_type_deltaPositions_constr_2, SEQUENCE_OF_constraint },
	asn_MBR_deltaPositions_2,
	1,	/* Single element */
	&asn_SPC_deltaPositions_specs_2	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_deltaPositionsWithAltitude_4[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_DeltaReferencePosition,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_deltaPositionsWithAltitude_tags_4[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_deltaPositionsWithAltitude_specs_4 = {
	sizeof(struct PolygonalLine__deltaPositionsWithAltitude),
	offsetof(struct PolygonalLine__deltaPositionsWithAltitude, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_deltaPositionsWithAltitude_4 = {
	"deltaPositionsWithAltitude",
	"deltaPositionsWithAltitude",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_deltaPositionsWithAltitude_tags_4,
	sizeof(asn_DEF_deltaPositionsWithAltitude_tags_4)
		/sizeof(asn_DEF_deltaPositionsWithAltitude_tags_4[0]) - 1, /* 1 */
	asn_DEF_deltaPositionsWithAltitude_tags_4,	/* Same as above */
	sizeof(asn_DEF_deltaPositionsWithAltitude_tags_4)
		/sizeof(asn_DEF_deltaPositionsWithAltitude_tags_4[0]), /* 2 */
	{ &asn_OER_type_deltaPositionsWithAltitude_constr_4, &asn_PER_type_deltaPositionsWithAltitude_constr_4, SEQUENCE_OF_constraint },
	asn_MBR_deltaPositionsWithAltitude_4,
	1,	/* Single element */
	&asn_SPC_deltaPositionsWithAltitude_specs_4	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_absolutePositions_6[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_AbsolutePosition,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_absolutePositions_tags_6[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_absolutePositions_specs_6 = {
	sizeof(struct PolygonalLine__absolutePositions),
	offsetof(struct PolygonalLine__absolutePositions, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_absolutePositions_6 = {
	"absolutePositions",
	"absolutePositions",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_absolutePositions_tags_6,
	sizeof(asn_DEF_absolutePositions_tags_6)
		/sizeof(asn_DEF_absolutePositions_tags_6[0]) - 1, /* 1 */
	asn_DEF_absolutePositions_tags_6,	/* Same as above */
	sizeof(asn_DEF_absolutePositions_tags_6)
		/sizeof(asn_DEF_absolutePositions_tags_6[0]), /* 2 */
	{ &asn_OER_type_absolutePositions_constr_6, &asn_PER_type_absolutePositions_constr_6, SEQUENCE_OF_constraint },
	asn_MBR_absolutePositions_6,
	1,	/* Single element */
	&asn_SPC_absolutePositions_specs_6	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_absolutePositionsWithAltitude_8[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_AbsolutePositionWAltitude,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_absolutePositionsWithAltitude_tags_8[] = {
	(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_absolutePositionsWithAltitude_specs_8 = {
	sizeof(struct PolygonalLine__absolutePositionsWithAltitude),
	offsetof(struct PolygonalLine__absolutePositionsWithAltitude, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_absolutePositionsWithAltitude_8 = {
	"absolutePositionsWithAltitude",
	"absolutePositionsWithAltitude",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_absolutePositionsWithAltitude_tags_8,
	sizeof(asn_DEF_absolutePositionsWithAltitude_tags_8)
		/sizeof(asn_DEF_absolutePositionsWithAltitude_tags_8[0]) - 1, /* 1 */
	asn_DEF_absolutePositionsWithAltitude_tags_8,	/* Same as above */
	sizeof(asn_DEF_absolutePositionsWithAltitude_tags_8)
		/sizeof(asn_DEF_absolutePositionsWithAltitude_tags_8[0]), /* 2 */
	{ &asn_OER_type_absolutePositionsWithAltitude_constr_8, &asn_PER_type_absolutePositionsWithAltitude_constr_8, SEQUENCE_OF_constraint },
	asn_MBR_absolutePositionsWithAltitude_8,
	1,	/* Single element */
	&asn_SPC_absolutePositionsWithAltitude_specs_8	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_PolygonalLine_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PolygonalLine, choice.deltaPositions),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		0,
		&asn_DEF_deltaPositions_2,
		0,
		{ &asn_OER_memb_deltaPositions_constr_2, &asn_PER_memb_deltaPositions_constr_2,  memb_deltaPositions_constraint_1 },
		0, 0, /* No default value */
		"deltaPositions"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PolygonalLine, choice.deltaPositionsWithAltitude),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		0,
		&asn_DEF_deltaPositionsWithAltitude_4,
		0,
		{ &asn_OER_memb_deltaPositionsWithAltitude_constr_4, &asn_PER_memb_deltaPositionsWithAltitude_constr_4,  memb_deltaPositionsWithAltitude_constraint_1 },
		0, 0, /* No default value */
		"deltaPositionsWithAltitude"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PolygonalLine, choice.absolutePositions),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		0,
		&asn_DEF_absolutePositions_6,
		0,
		{ &asn_OER_memb_absolutePositions_constr_6, &asn_PER_memb_absolutePositions_constr_6,  memb_absolutePositions_constraint_1 },
		0, 0, /* No default value */
		"absolutePositions"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PolygonalLine, choice.absolutePositionsWithAltitude),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		0,
		&asn_DEF_absolutePositionsWithAltitude_8,
		0,
		{ &asn_OER_memb_absolutePositionsWithAltitude_constr_8, &asn_PER_memb_absolutePositionsWithAltitude_constr_8,  memb_absolutePositionsWithAltitude_constraint_1 },
		0, 0, /* No default value */
		"absolutePositionsWithAltitude"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_PolygonalLine_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* deltaPositions */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* deltaPositionsWithAltitude */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* absolutePositions */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* absolutePositionsWithAltitude */
};
asn_CHOICE_specifics_t asn_SPC_PolygonalLine_specs_1 = {
	sizeof(struct PolygonalLine),
	offsetof(struct PolygonalLine, _asn_ctx),
	offsetof(struct PolygonalLine, present),
	sizeof(((struct PolygonalLine *)0)->present),
	asn_MAP_PolygonalLine_tag2el_1,
	4,	/* Count of tags in the map */
	0, 0,
	4	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_PolygonalLine = {
	"PolygonalLine",
	"PolygonalLine",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_PolygonalLine_constr_1, &asn_PER_type_PolygonalLine_constr_1, CHOICE_constraint },
	asn_MBR_PolygonalLine_1,
	4,	/* Elements count */
	&asn_SPC_PolygonalLine_specs_1	/* Additional specs */
};

