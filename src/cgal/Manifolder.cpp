#include "Manifolder.h"

#include "utils/Types.h"
#include "utils/CoordGraph.h"
#include "utils/Cleaner.h"
#include "utils/MeshTools.h"

namespace meshlib {
namespace cgal {

using FaceIds = std::set<CoordinateId>;
using EdgeIds = std::set<CoordinateId>;
using VertexId = std::size_t;
using PointToCoordIdMap = std::map<K::Point_3, CoordinateId>;
using VertexIdToCoordinateIdMap = std::map<VertexId, CoordinateId>;

using namespace polyhedronTools;

Elements buildExternalFaces(const Elements& es)
{
	std::map<FaceIds, CoordinateIds> externalFaces;
	for (auto const& e : es) {
		ElementId eId = &e - &es.front();
		if (e.isTetrahedron()) {
			for (std::size_t f = 0; f < 4; f++) {
				FaceIds faceIds = { e.vertices[f % 4], e.vertices[(f + 1) % 4], e.vertices[(f + 2) % 4] };
				auto it = externalFaces.find(faceIds);
				if (it == externalFaces.end()) {
					externalFaces.emplace(faceIds, CoordinateIds(faceIds.begin(), faceIds.end()));
				}
				else {
					externalFaces.erase(it);
				}
			}
		}
		else if (e.isTriangle()) {
			externalFaces.emplace(FaceIds(e.vertices.begin(), e.vertices.end()), e.vertices);
		}
		else {
			std::runtime_error("Unsupported element type.");
		}
	}

	Elements res;
	res.reserve(externalFaces.size());
	for (auto const& kv : externalFaces) {
		res.push_back(Element(kv.second, Element::Type::Surface));
	}

	return res;
}

Groups buildExternalFacesGroups(const Mesh& m)
{
	Groups res(m.groups.size());
	for (auto const& g : m.groups) {
		GroupId gId = &g - &m.groups.front();
		res[gId].elements = buildExternalFaces(g.elements);
	}
	return res;
}

Manifolder::Manifolder(const Mesh& m) :
	grid_{m.grid},
	groupsSize_{m.groups.size()}
{
	const auto externalFaces{ buildExternalFacesGroups(m) };
	for (auto& g : externalFaces) {
		if (g.elements.empty()) {
			continue;
		}
		const auto gId{ &g - &externalFaces.front() };
		auto ps{ buildClosedAndOpenPolyhedrons(m.coordinates, g.elements) };
		closed_[gId] = std::move(ps.first);
		open_[gId] = std::move(ps.second);
	}
}

Mesh Manifolder::buildFromGroupToPolyhedronMap(const std::map<GroupId, Polyhedron>& mTP) const
{
	Mesh r{grid_};
	r.groups.resize(groupsSize_);
	for (const auto& [gId, pol] : mTP) {
		r.groups[gId] = buildGroupFromPolyhedron(r, pol);
	}
	return r;
}

Mesh Manifolder::getOpenSurfacesMesh() const
{
	return buildFromGroupToPolyhedronMap(open_);
}

Mesh Manifolder::getClosedSurfacesMesh() const
{
	return buildFromGroupToPolyhedronMap(closed_);
}

Mesh Manifolder::getSurfacesMesh() const
{
	Mesh m = getClosedSurfacesMesh();
	utils::meshTools::mergeMesh(m, getOpenSurfacesMesh());
	utils::Cleaner::cleanCoords(m);
	return m;
}

}
}
