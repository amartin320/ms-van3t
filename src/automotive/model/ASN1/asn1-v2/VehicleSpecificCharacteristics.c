/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_14906_EfcDsrcApplication.asn"
 * 	`asn1c -fcompound-names`
 */

#include "VehicleSpecificCharacteristics.h"

static asn_TYPE_member_t asn_MBR_VehicleSpecificCharacteristics_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleSpecificCharacteristics, environmentalCharacteristics),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_EnvironmentalCharacteristics,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"environmentalCharacteristics"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleSpecificCharacteristics, engineCharacteristics),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_EngineCharacteristics,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"engineCharacteristics"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleSpecificCharacteristics, descriptiveCharacteristics),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DescriptiveCharacteristics,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"descriptiveCharacteristics"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleSpecificCharacteristics, futureCharacteristics),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FutureCharacteristics,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"futureCharacteristics"
		},
};
static const ber_tlv_tag_t asn_DEF_VehicleSpecificCharacteristics_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_VehicleSpecificCharacteristics_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* environmentalCharacteristics */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* engineCharacteristics */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* descriptiveCharacteristics */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* futureCharacteristics */
};
static asn_SEQUENCE_specifics_t asn_SPC_VehicleSpecificCharacteristics_specs_1 = {
	sizeof(struct VehicleSpecificCharacteristics),
	offsetof(struct VehicleSpecificCharacteristics, _asn_ctx),
	asn_MAP_VehicleSpecificCharacteristics_tag2el_1,
	4,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_VehicleSpecificCharacteristics = {
	"VehicleSpecificCharacteristics",
	"VehicleSpecificCharacteristics",
	&asn_OP_SEQUENCE,
	asn_DEF_VehicleSpecificCharacteristics_tags_1,
	sizeof(asn_DEF_VehicleSpecificCharacteristics_tags_1)
		/sizeof(asn_DEF_VehicleSpecificCharacteristics_tags_1[0]), /* 1 */
	asn_DEF_VehicleSpecificCharacteristics_tags_1,	/* Same as above */
	sizeof(asn_DEF_VehicleSpecificCharacteristics_tags_1)
		/sizeof(asn_DEF_VehicleSpecificCharacteristics_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_VehicleSpecificCharacteristics_1,
	4,	/* Elements count */
	&asn_SPC_VehicleSpecificCharacteristics_specs_1	/* Additional specs */
};

