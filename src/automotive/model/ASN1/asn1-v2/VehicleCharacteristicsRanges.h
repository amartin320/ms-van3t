/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "../IVIM-ASN1-files/asn1_IS_ISO_TS_19321_IVI.asn"
 * 	`asn1c -fcompound-names`
 */

#ifndef	_VehicleCharacteristicsRanges_H_
#define	_VehicleCharacteristicsRanges_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ComparisonOperator.h"
#include "NativeInteger.h"
#include "VehicleDimensions.h"
#include "VehicleWeightLimits.h"
#include "AxleWeightLimits.h"
#include "PassengerCapacity.h"
#include "ExhaustEmissionValues.h"
#include "DieselEmissionValues.h"
#include "SoundLevel.h"
#include "constr_CHOICE.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VehicleCharacteristicsRanges__limits_PR {
	VehicleCharacteristicsRanges__limits_PR_NOTHING,	/* No components present */
	VehicleCharacteristicsRanges__limits_PR_numberOfAxles,
	VehicleCharacteristicsRanges__limits_PR_vehicleDimensions,
	VehicleCharacteristicsRanges__limits_PR_vehicleWeightLimits,
	VehicleCharacteristicsRanges__limits_PR_axleWeightLimits,
	VehicleCharacteristicsRanges__limits_PR_passengerCapacity,
	VehicleCharacteristicsRanges__limits_PR_exhaustEmissionValues,
	VehicleCharacteristicsRanges__limits_PR_dieselEmissionValues,
	VehicleCharacteristicsRanges__limits_PR_soundLevel
	/* Extensions may appear below */
	
} VehicleCharacteristicsRanges__limits_PR;

/* VehicleCharacteristicsRanges */
typedef struct VehicleCharacteristicsRanges {
	ComparisonOperator_t	 comparisonOperator;
	struct VehicleCharacteristicsRanges__limits {
		VehicleCharacteristicsRanges__limits_PR present;
		union VehicleCharacteristicsRanges__limits_u {
			long	 numberOfAxles;
			VehicleDimensions_t	 vehicleDimensions;
			VehicleWeightLimits_t	 vehicleWeightLimits;
			AxleWeightLimits_t	 axleWeightLimits;
			PassengerCapacity_t	 passengerCapacity;
			ExhaustEmissionValues_t	 exhaustEmissionValues;
			DieselEmissionValues_t	 dieselEmissionValues;
			SoundLevel_t	 soundLevel;
			/*
			 * This type is extensible,
			 * possible extensions are below.
			 */
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} limits;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VehicleCharacteristicsRanges_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VehicleCharacteristicsRanges;
extern asn_SEQUENCE_specifics_t asn_SPC_VehicleCharacteristicsRanges_specs_1;
extern asn_TYPE_member_t asn_MBR_VehicleCharacteristicsRanges_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _VehicleCharacteristicsRanges_H_ */
#include "asn_internal.h"
