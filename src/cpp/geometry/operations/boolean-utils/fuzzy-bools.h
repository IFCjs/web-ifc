#pragma once

#include "geometry.h"
#include "shared-position.h"
#include "clip-mesh.h"

namespace fuzzybools
{
	inline Geometry Subtract(const Geometry& A, const Geometry& B)
	{
		fuzzybools::SharedPosition sp;
		sp.Construct(A, B);

		auto bvh1 = fuzzybools::MakeBVH(A);
		auto bvh2 = fuzzybools::MakeBVH(B);

		auto geom = Normalize(sp);

        #ifdef CSG_DEBUG_OUTPUT
			DumpGeometry(geom, L"Post-normalize.obj");
		#endif

		return fuzzybools::clipSubtract(geom, bvh1, bvh2);
	}

	inline Geometry Union(const Geometry& A, const Geometry& B)
	{
		fuzzybools::SharedPosition sp;
		sp.Construct(A, B);

		auto bvh1 = fuzzybools::MakeBVH(A);
		auto bvh2 = fuzzybools::MakeBVH(B);

		auto geom = Normalize(sp);

		return fuzzybools::clipJoin(geom, bvh1, bvh2);
	}
}