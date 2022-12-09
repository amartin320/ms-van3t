/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#include "VehicleCharacteristicsRanges.h"

static int
memb_numberOfAxles_constraint_3(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 7)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_memb_numberOfAxles_constr_4 CC_NOTUSED = {
	{ 1, 1 }	/* (0..7) */,
	-1};
static asn_per_constraints_t asn_PER_memb_numberOfAxles_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_limits_constr_3 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_limits_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  0,  7 }	/* (0..7,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_limits_3[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges__limits, choice.numberOfAxles),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_numberOfAxles_constr_4, &asn_PER_memb_numberOfAxles_constr_4,  memb_numberOfAxles_constraint_3 },
		0, 0, /* No default value */
		"numberOfAxles"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges__limits, choice.vehicleDimensions),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleDimensions,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"vehicleDimensions"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges__limits, choice.vehicleWeightLimits),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VehicleWeightLimits,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"vehicleWeightLimits"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges__limits, choice.axleWeightLimits),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_AxleWeightLimits,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"axleWeightLimits"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges__limits, choice.passengerCapacity),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PassengerCapacity,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"passengerCapacity"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges__limits, choice.exhaustEmissionValues),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ExhaustEmissionValues,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"exhaustEmissionValues"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges__limits, choice.dieselEmissionValues),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DieselEmissionValues,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"dieselEmissionValues"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges__limits, choice.soundLevel),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SoundLevel,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"soundLevel"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_limits_tag2el_3[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* numberOfAxles */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* vehicleDimensions */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* vehicleWeightLimits */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* axleWeightLimits */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* passengerCapacity */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* exhaustEmissionValues */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* dieselEmissionValues */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 } /* soundLevel */
};
static asn_CHOICE_specifics_t asn_SPC_limits_specs_3 = {
	sizeof(struct VehicleCharacteristicsRanges__limits),
	offsetof(struct VehicleCharacteristicsRanges__limits, _asn_ctx),
	offsetof(struct VehicleCharacteristicsRanges__limits, present),
	sizeof(((struct VehicleCharacteristicsRanges__limits *)0)->present),
	asn_MAP_limits_tag2el_3,
	8,	/* Count of tags in the map */
	0, 0,
	8	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_limits_3 = {
	"limits",
	"limits",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_limits_constr_3, &asn_PER_type_limits_constr_3, CHOICE_constraint },
	asn_MBR_limits_3,
	8,	/* Elements count */
	&asn_SPC_limits_specs_3	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_VehicleCharacteristicsRanges_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges, comparisonOperator),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ComparisonOperator,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"comparisonOperator"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleCharacteristicsRanges, limits),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_limits_3,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"limits"
		},
};
static const ber_tlv_tag_t asn_DEF_VehicleCharacteristicsRanges_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_VehicleCharacteristicsRanges_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* comparisonOperator */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* limits */
};
asn_SEQUENCE_specifics_t asn_SPC_VehicleCharacteristicsRanges_specs_1 = {
	sizeof(struct VehicleCharacteristicsRanges),
	offsetof(struct VehicleCharacteristicsRanges, _asn_ctx),
	asn_MAP_VehicleCharacteristicsRanges_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_VehicleCharacteristicsRanges = {
	"VehicleCharacteristicsRanges",
	"VehicleCharacteristicsRanges",
	&asn_OP_SEQUENCE,
	asn_DEF_VehicleCharacteristicsRanges_tags_1,
	sizeof(asn_DEF_VehicleCharacteristicsRanges_tags_1)
		/sizeof(asn_DEF_VehicleCharacteristicsRanges_tags_1[0]), /* 1 */
	asn_DEF_VehicleCharacteristicsRanges_tags_1,	/* Same as above */
	sizeof(asn_DEF_VehicleCharacteristicsRanges_tags_1)
		/sizeof(asn_DEF_VehicleCharacteristicsRanges_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_VehicleCharacteristicsRanges_1,
	2,	/* Elements count */
	&asn_SPC_VehicleCharacteristicsRanges_specs_1	/* Additional specs */
};

