#pragma once

#include "navislam_global.h"
#include "matlib.h"
#include <math.h>

namespace navicore
{

#define USHORT unsigned short

	const float kMinProbability = 0.10f;
	const float kMaxProbability = 0.90f;
	const float khitProbability = 0.55f;
	const USHORT kUnknownProbabilityValue = 0;
	const USHORT kUpdateMarker = 1u << 15;

    template<typename T>
    T static clamp(const T value, const T min, const T max) {
        if(value > max) {
            return max;
        }
        if(value < min) {
            return min;
        }
        return value;
    }

    class NAVISLAMSHARED_EXPORT ProbabilityValue16
	{
	public:
		static ProbabilityValue16* getInstance();
	   ~ProbabilityValue16(void);

	  inline float Odds(float probability) {
		return probability / (1.f - probability);
	  }

	   inline float ProbabilityFromOdds(const float odds) {
		 return odds / (odds + 1.f);
	  }

	   inline float ClampProbability(const float probability) {
         return clamp((float)probability, kMinProbability, kMaxProbability);
	   }

	   inline USHORT ProbabilityToValue(const float probability) {
        const int value = floor((ClampProbability(probability) - kMinProbability) *
								     (32766.f / (kMaxProbability - kMinProbability))) + 1;
		return value;
	  }

	    float SlowValueToProbability(const USHORT value);
		std::vector<float>*  PrecomputeValueToProbability() ;

		// Converts a uint16 (which may or may not have the update marker set) to a
		// probability in the range [kMinProbability, kMaxProbability].
		inline float ValueToProbability(const USHORT value) {
		  return (*kValueToProbability)[value];
		}

		std::vector<USHORT> ComputeLookupTableToApplyOdds(float odds);

		std::vector<USHORT> hit_table;
		USHORT hitValue;

	private:
		 ProbabilityValue16(void);
	     std::vector<float>*  kValueToProbability;
		 static ProbabilityValue16* _pInstance;
         static bool instanceFlag;
	};

}
