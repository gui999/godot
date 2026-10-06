/**************************************************************************/
/*  jolt_custom_voxel_grid_shape.cpp                                      */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "jolt_custom_voxel_grid_shape.h"

#include <Jolt/Geometry/AABox4.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iterator>
#include <mutex>
#include <unordered_map>

namespace {

constexpr int FACE_STEP[6][3] = { { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 } };

JPH::uint bits_for(int p_count) {
	JPH::uint bits = 0;
	while ((int64_t(1) << bits) < int64_t(p_count)) {
		bits++;
	}
	return bits;
}

// Whether a neighbour's boundary layer covers a face's rectangle (u0..u1, v0..v1, voxels).
bool layer_covers(const uint32_t *p_layer, int p_u0, int p_u1, int p_v0, int p_v1) {
	const uint32_t bits = (p_u1 - p_u0 >= 32 ? 0xffffffffu : ((1u << (p_u1 - p_u0)) - 1u)) << p_u0;
	for (int v = p_v0; v < p_v1; v++) {
		if ((p_layer[v] & bits) != bits) {
			return false;
		}
	}
	return true;
}

JPH::Shape *construct_voxel_grid() {
	return nullptr;
}

} // namespace

// Passes a box's contacts on, with none pushing through a face that solid covers. Every hit Jolt
// finds is an overlap and stays one (Godot's motion casts and shape queries only ask whether there is
// any). A hit whose normal (the box's outward normal, the negated penetration axis) has a component
// into a covered face is answered again on the box's uncovered faces: the face the other shape
// penetrates least, with its exact depth from the shape's support (the separating-axis test on the
// box's face axes). Dropped only when every face is covered or the shape is clear of that face.
class JoltVoxelGridCoveredFaceCollector final : public JPH::CollideShapeCollector {
public:
	JoltVoxelGridCoveredFaceCollector(JPH::CollideShapeCollector &p_inner, JPH::Mat44Arg p_grid_transform, const JoltCustomVoxelGridShape &p_grid, const JPH::ConvexShape::Support &p_support, JPH::Mat44Arg p_into, float p_max_separation) :
			JPH::CollideShapeCollector(p_inner), inner(p_inner), grid_transform(p_grid_transform), grid(p_grid), support(p_support), into(p_into), max_separation(p_max_separation) {}

	void box(int p_cell, int p_box) {
		cell = p_cell;
		box_index = p_box;
		known = false;
	}

	void AddHit(const JPH::CollideShapeResult &p_hit) override {
		if (!known) {
			covered = grid.covered_faces(cell, box_index);
			known = true;
		}
		if (covered == 0 || !into_covered(p_hit)) {
			inner.AddHit(p_hit);
			sync();
			return;
		}
		// The uncovered face the shape penetrates least, in the grid's frame.
		const JPH::AABox bounds = grid.box_bounds(cell, box_index);
		int best = -1;
		float best_depth = FLT_MAX;
		JPH::Vec3 best_point = JPH::Vec3::sZero();
		for (int face = 0; face < 6; face++) {
			if (covered & (1 << face)) {
				continue;
			}
			const int axis = face / 2;
			const float sign = (face & 1) ? 1.0f : -1.0f;
			JPH::Vec3 outward = JPH::Vec3::sZero();
			outward.SetComponent(axis, sign);
			// The shape's point deepest past the face's plane: its support opposite the face's normal.
			const JPH::Vec3 point = into * support.GetSupport(into.Multiply3x3Transposed(-outward));
			const float plane = sign > 0 ? bounds.mMax[axis] : -bounds.mMin[axis];
			const float depth = plane - point.Dot(outward);
			if (depth < best_depth) {
				best_depth = depth;
				best = face;
				best_point = point;
			}
		}
		if (best < 0 || best_depth < -max_separation) {
			sync();
			return;
		}
		JPH::Vec3 outward = JPH::Vec3::sZero();
		outward.SetComponent(best / 2, (best & 1) ? 1.0f : -1.0f);
		JPH::CollideShapeResult answered = p_hit;
		answered.mPenetrationAxis = -grid_transform.Multiply3x3(outward);
		answered.mPenetrationDepth = best_depth;
		answered.mContactPointOn1 = grid_transform * best_point;
		answered.mContactPointOn2 = grid_transform * (best_point + outward * best_depth);
		if (!answered.mShape2Face.empty()) {
			bounds.GetSupportingFace(-outward, answered.mShape2Face);
			for (JPH::Vec3 &vertex : answered.mShape2Face) {
				vertex = grid_transform * vertex;
			}
		}
		inner.AddHit(answered);
		sync();
	}

private:
	// Whether the hit's normal has a component into a covered face.
	bool into_covered(const JPH::CollideShapeResult &p_hit) const {
		const JPH::Vec3 normal = -grid_transform.Multiply3x3Transposed(p_hit.mPenetrationAxis).NormalizedOr(JPH::Vec3::sZero());
		for (int a = 0; a < 3; a++) {
			const float c = normal[a];
			if (std::abs(c) >= 1.0e-4f && (covered & (1 << (a * 2 + (c > 0 ? 1 : 0))))) {
				return true;
			}
		}
		return false;
	}

	void sync() {
		if (inner.ShouldEarlyOut()) {
			ForceEarlyOut();
		} else if (inner.GetEarlyOutFraction() < GetEarlyOutFraction()) {
			UpdateEarlyOutFraction(inner.GetEarlyOutFraction());
		}
	}

	JPH::CollideShapeCollector &inner;
	JPH::Mat44 grid_transform;
	const JoltCustomVoxelGridShape &grid;
	const JPH::ConvexShape::Support &support;
	JPH::Mat44 into;
	float max_separation = 0.0f;
	int cell = 0;
	int box_index = 0;
	bool known = false;
	uint8_t covered = 0;
};

namespace {

// Derives a module's query data from its boxes.
void derive_module(JoltCustomVoxelGridShape::Module &module, const float *p_boxes, int p_box_count, float p_cell_meters, int p_cell_voxels) {
	module.source.assign(p_boxes, p_boxes + size_t(p_box_count) * 6);
	module.box_count = p_box_count;
	const size_t groups = (size_t(p_box_count) + 3) / 4;
	module.groups.assign(groups * 24, 0.0f);
	for (size_t g = 0; g < groups; g++) {
		float *group = &module.groups[g * 24];
		for (int lane = 0; lane < 4; lane++) {
			const size_t b = g * 4 + lane;
			if (b >= size_t(p_box_count)) {
				for (int a = 0; a < 3; a++) {
					group[a * 4 + lane] = 1e30f;
					group[12 + a * 4 + lane] = -1e30f;
				}
				continue;
			}
			for (int a = 0; a < 3; a++) {
				group[a * 4 + lane] = p_boxes[b * 6 + a];
				group[12 + a * 4 + lane] = p_boxes[b * 6 + 3 + a];
			}
		}
	}
	module.covered_faces.assign(size_t(p_box_count), 0);
	module.boundary_faces.assign(size_t(p_box_count), 0);

	// Face cover needs the boxes on the voxel grid; a module whose boxes are not on it covers nothing
	// and is covered by nothing of its own (its contacts are kept as they are).
	const int n = p_cell_voxels;
	const float voxel = p_cell_meters / float(n);
	module.voxel_boxes.assign(size_t(p_box_count) * 6, 0);
	for (int b = 0; b < p_box_count; b++) {
		for (int a = 0; a < 3; a++) {
			const float low = p_boxes[b * 6 + a] / voxel, high = p_boxes[b * 6 + 3 + a] / voxel;
			const int low_voxel = int(std::lround(low)), high_voxel = int(std::lround(high));
			if (std::abs(low - float(low_voxel)) > 1.0e-3f || std::abs(high - float(high_voxel)) > 1.0e-3f || low_voxel < 0 || high_voxel > n || high_voxel <= low_voxel) {
				module.voxel_boxes.clear();
				return;
			}
			module.voxel_boxes[b * 6 + a] = low_voxel;
			module.voxel_boxes[b * 6 + 3 + a] = high_voxel - low_voxel;
		}
	}
	std::vector<uint8_t> solid(size_t(n) * n * n, 0);
	auto at = [n](int x, int y, int z) { return (size_t(y) * n + z) * n + x; };
	for (int b = 0; b < p_box_count; b++) {
		const int *box = &module.voxel_boxes[b * 6];
		for (int y = box[1]; y < box[1] + box[4]; y++) {
			for (int z = box[2]; z < box[2] + box[5]; z++) {
				for (int x = box[0]; x < box[0] + box[3]; x++) {
					solid[at(x, y, z)] = 1;
				}
			}
		}
	}
	module.boundary_layers.assign(6 * size_t(n), 0);
	for (int axis = 0; axis < 3; axis++) {
		const int u = (axis + 1) % 3, v = (axis + 2) % 3;
		for (int side = 0; side < 2; side++) {
			const int layer = side == 0 ? 0 : n - 1;
			uint32_t *rows = &module.boundary_layers[(axis * 2 + side) * size_t(n)];
			for (int j = 0; j < n; j++) {
				for (int i = 0; i < n; i++) {
					int p[3];
					p[axis] = layer;
					p[u] = i;
					p[v] = j;
					if (solid[at(p[0], p[1], p[2])]) {
						rows[j] |= 1u << i;
					}
				}
			}
		}
	}
	for (int b = 0; b < p_box_count; b++) {
		const int *box = &module.voxel_boxes[b * 6];
		const int low[3] = { box[0], box[1], box[2] }, size[3] = { box[3], box[4], box[5] };
		for (int axis = 0; axis < 3; axis++) {
			const int u = (axis + 1) % 3, v = (axis + 2) % 3;
			for (int side = 0; side < 2; side++) {
				const int face = axis * 2 + side;
				const int layer = side == 0 ? low[axis] - 1 : low[axis] + size[axis];
				if (layer < 0 || layer >= n) {
					module.boundary_faces[b] |= 1 << face;
					continue;
				}
				bool face_covered = true;
				for (int j = low[v]; j < low[v] + size[v] && face_covered; j++) {
					for (int i = low[u]; i < low[u] + size[u] && face_covered; i++) {
						int p[3];
						p[axis] = layer;
						p[u] = i;
						p[v] = j;
						face_covered = solid[at(p[0], p[1], p[2])] != 0;
					}
				}
				if (face_covered) {
					module.covered_faces[b] |= 1 << face;
				}
			}
		}
	}
}

uint64_t hash_boxes(const float *p_boxes, int p_box_count, float p_cell_meters, int p_cell_voxels) {
	uint64_t hash = 14695981039346656037ull;
	auto mix = [&hash](const void *p_bytes, size_t p_size) {
		const uint8_t *bytes = static_cast<const uint8_t *>(p_bytes);
		for (size_t i = 0; i < p_size; i++) {
			hash = (hash ^ bytes[i]) * 1099511628211ull;
		}
	};
	mix(&p_cell_meters, sizeof(float));
	mix(&p_cell_voxels, sizeof(int));
	mix(p_boxes, size_t(p_box_count) * 6 * sizeof(float));
	return hash;
}

std::mutex module_cache_mutex;
std::unordered_multimap<uint64_t, JPH::RefConst<JoltCustomVoxelGridShape::Module>> module_cache;
size_t module_cache_sweep_at = 4096;

} // namespace

JPH::RefConst<JoltCustomVoxelGridShape::Module> JoltCustomVoxelGridShape::get_module(const float *p_boxes, int p_box_count, float p_cell_meters, int p_cell_voxels) {
	const uint64_t hash = hash_boxes(p_boxes, p_box_count, p_cell_meters, p_cell_voxels);
	{
		std::lock_guard<std::mutex> lock(module_cache_mutex);
		auto range = module_cache.equal_range(hash);
		for (auto it = range.first; it != range.second; ++it) {
			const Module &cached = *it->second;
			if (cached.box_count == p_box_count && std::equal(cached.source.begin(), cached.source.end(), p_boxes)) {
				return it->second;
			}
		}
	}
	JPH::Ref<Module> module = new Module();
	derive_module(*module, p_boxes, p_box_count, p_cell_meters, p_cell_voxels);
	std::lock_guard<std::mutex> lock(module_cache_mutex);
	// Modules no shape holds any more (hole-carved cells replaced since) leave the cache when it grows.
	if (module_cache.size() >= module_cache_sweep_at) {
		for (auto it = module_cache.begin(); it != module_cache.end();) {
			it = it->second->GetRefCount() == 1 ? module_cache.erase(it) : std::next(it);
		}
		module_cache_sweep_at = std::max<size_t>(4096, module_cache.size() * 2);
	}
	module_cache.emplace(hash, module);
	return module;
}

JoltCustomVoxelGridShape::JoltCustomVoxelGridShape(Data &&p_data) :
		JPH::Shape(JPH::EShapeType::User3, JoltCustomShapeSubType::VOXEL_GRID), data(std::move(p_data)) {
	const int cells = data.size[0] * data.size[1] * data.size[2];
	cell_bits = bits_for(std::max(cells, 1));
	// The most boxes a cell can hold (one a voxel), so a module added later fits the same IDs.
	box_bits = bits_for(data.cell_voxels * data.cell_voxels * data.cell_voxels);
	// The whole grid, occupied or not, so a cell update never moves the bounds.
	bounds = JPH::AABox(data.origin, data.origin + JPH::Vec3(float(data.size[0]), float(data.size[1]), float(data.size[2])) * data.cell_meters);
}

void JoltCustomVoxelGridShape::set_cell(int p_padded_cell, int p_module) {
	JPH_ASSERT(p_padded_cell >= 0 && p_padded_cell < int(data.padded_modules.size()));
	JPH_ASSERT(p_module >= -1 && p_module < int(data.modules.size()));
	data.padded_modules[size_t(p_padded_cell)] = p_module;
}

bool JoltCustomVoxelGridShape::add_module(const JPH::RefConst<Module> &p_module) {
	if (p_module->box_count > (1 << box_bits)) {
		return false;
	}
	data.modules.push_back(p_module);
	return true;
}

const JPH::PhysicsMaterial *JoltCustomVoxelGridShape::GetMaterial(const JPH::SubShapeID &p_sub_shape_id) const {
	return JPH::PhysicsMaterial::sDefault;
}

void JoltCustomVoxelGridShape::cell_coordinates(int p_cell, int &r_i, int &r_j, int &r_k) const {
	r_i = p_cell % data.size[0];
	r_k = (p_cell / data.size[0]) % data.size[2];
	r_j = p_cell / (data.size[0] * data.size[2]);
}

int JoltCustomVoxelGridShape::module_at(int p_cell) const {
	int i, j, k;
	cell_coordinates(p_cell, i, j, k);
	return data.padded_modules[padded_index(i, j, k)];
}

int JoltCustomVoxelGridShape::get_box_count() const {
	int count = 0;
	for (int cell = 0; cell < data.size[0] * data.size[1] * data.size[2]; cell++) {
		const int module = module_at(cell);
		if (module >= 0) {
			count += data.modules[module]->box_count;
		}
	}
	return count;
}

JPH::Shape::Stats JoltCustomVoxelGridShape::GetStats() const {
	size_t bytes = sizeof(*this) + data.padded_modules.capacity() * sizeof(int32_t);
	for (const JPH::RefConst<Module> &shared : data.modules) {
		const Module &module = *shared;
		bytes += module.groups.capacity() * sizeof(float) + module.covered_faces.capacity() + module.boundary_faces.capacity() + module.boundary_layers.capacity() * sizeof(uint32_t) + module.voxel_boxes.capacity() * sizeof(int);
	}
	return Stats(bytes, 0);
}

JPH::AABox JoltCustomVoxelGridShape::box_bounds(int p_cell, int p_box) const {
	int i, j, k;
	cell_coordinates(p_cell, i, j, k);
	const Module &module = *data.modules[module_at(p_cell)];
	const float *group = &module.groups[size_t(p_box / 4) * 24];
	const int lane = p_box % 4;
	const JPH::Vec3 corner = data.origin + JPH::Vec3(float(i), float(j), float(k)) * data.cell_meters;
	return JPH::AABox(corner + JPH::Vec3(group[lane], group[4 + lane], group[8 + lane]), corner + JPH::Vec3(group[12 + lane], group[16 + lane], group[20 + lane]));
}

uint8_t JoltCustomVoxelGridShape::covered_faces(int p_cell, int p_box) const {
	const Module &module = *data.modules[module_at(p_cell)];
	if (module.voxel_boxes.empty()) {
		return 0;
	}
	uint8_t covered = module.covered_faces[p_box];
	const uint8_t boundary = module.boundary_faces[p_box];
	if (boundary == 0) {
		return covered;
	}
	int i, j, k;
	cell_coordinates(p_cell, i, j, k);
	const int *box = &module.voxel_boxes[size_t(p_box) * 6];
	const int low[3] = { box[0], box[1], box[2] }, size[3] = { box[3], box[4], box[5] };
	const int n = data.cell_voxels;
	for (int face = 0; face < 6; face++) {
		if (!(boundary & (1 << face))) {
			continue;
		}
		const int *step = FACE_STEP[face];
		const int neighbour = data.padded_modules[padded_index(i + step[0], j + step[1], k + step[2])];
		if (neighbour < 0) {
			continue;
		}
		const Module &other = *data.modules[neighbour];
		if (other.boundary_layers.empty()) {
			continue;
		}
		const int axis = face / 2, u = (axis + 1) % 3, v = (axis + 2) % 3;
		// The neighbour's layer on the opposite side.
		if (layer_covers(&other.boundary_layers[size_t(face ^ 1) * n], low[u], low[u] + size[u], low[v], low[v] + size[v])) {
			covered |= 1 << face;
		}
	}
	return covered;
}

template <class F>
void JoltCustomVoxelGridShape::for_boxes(const JPH::AABox &p_bounds, F &&p_visit) const {
	const JPH::Vec3 from = (p_bounds.mMin - data.origin) / data.cell_meters, to = (p_bounds.mMax - data.origin) / data.cell_meters;
	const int i0 = std::max(0, int(std::floor(from.GetX()))), i1 = std::min(data.size[0] - 1, int(std::floor(to.GetX())));
	const int j0 = std::max(0, int(std::floor(from.GetY()))), j1 = std::min(data.size[1] - 1, int(std::floor(to.GetY())));
	const int k0 = std::max(0, int(std::floor(from.GetZ()))), k1 = std::min(data.size[2] - 1, int(std::floor(to.GetZ())));
	for (int j = j0; j <= j1; j++) {
		for (int k = k0; k <= k1; k++) {
			for (int i = i0; i <= i1; i++) {
				const int index = data.padded_modules[padded_index(i, j, k)];
				if (index < 0) {
					continue;
				}
				const Module &module = *data.modules[index];
				const int cell = cell_index(i, j, k);
				const JPH::Vec3 corner = data.origin + JPH::Vec3(float(i), float(j), float(k)) * data.cell_meters;
				const JPH::AABox local(p_bounds.mMin - corner, p_bounds.mMax - corner);
				const float *groups = module.groups.data();
				for (size_t g = 0; g < module.groups.size(); g += 24) {
					const float *group = groups + g;
					const JPH::Vec4 min_x = JPH::Vec4::sLoadFloat4((const JPH::Float4 *)group), min_y = JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 4)), min_z = JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 8));
					const JPH::Vec4 max_x = JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 12)), max_y = JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 16)), max_z = JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 20));
					int hits = JPH::AABox4VsBox(local, min_x, min_y, min_z, max_x, max_y, max_z).GetTrues();
					while (hits != 0) {
						const int lane = JPH::CountTrailingZeros(JPH::uint32(hits));
						hits &= hits - 1;
						const JPH::Vec3 low = corner + JPH::Vec3(group[lane], group[4 + lane], group[8 + lane]);
						const JPH::Vec3 high = corner + JPH::Vec3(group[12 + lane], group[16 + lane], group[20 + lane]);
						if (!p_visit(cell, int(g / 24) * 4 + lane, low, high)) {
							return;
						}
					}
				}
			}
		}
	}
}

void JoltCustomVoxelGridShape::decode(const JPH::SubShapeID &p_sub_shape_id, int &r_cell, int &r_box) const {
	int face;
	decode(p_sub_shape_id, r_cell, r_box, face);
}

void JoltCustomVoxelGridShape::decode(const JPH::SubShapeID &p_sub_shape_id, int &r_cell, int &r_box, int &r_face) const {
	JPH::SubShapeID remainder;
	r_cell = int(p_sub_shape_id.PopID(cell_bits, remainder));
	r_box = int(remainder.PopID(box_bits, remainder));
	r_face = int(remainder.PopID(FACE_BITS, remainder));
}

JPH::Vec3 JoltCustomVoxelGridShape::GetSurfaceNormal(const JPH::SubShapeID &p_sub_shape_id, JPH::Vec3Arg p_local_surface_position) const {
	int cell, box, face;
	decode(p_sub_shape_id, cell, box, face);
	if (face < UNKNOWN_FACE) {
		JPH::Vec3 normal = JPH::Vec3::sZero();
		normal.SetComponent(face / 2, (face & 1) ? 1.0f : -1.0f);
		return normal;
	}
	const JPH::AABox box_bounds_local = box_bounds(cell, box);
	// The face nearest the point.
	const JPH::Vec3 to_min = (p_local_surface_position - box_bounds_local.mMin).Abs(), to_max = (box_bounds_local.mMax - p_local_surface_position).Abs();
	float best = FLT_MAX;
	JPH::Vec3 normal = JPH::Vec3::sAxisY();
	for (int a = 0; a < 3; a++) {
		if (to_min[a] < best) {
			best = to_min[a];
			normal = JPH::Vec3::sZero();
			normal.SetComponent(a, -1.0f);
		}
		if (to_max[a] < best) {
			best = to_max[a];
			normal = JPH::Vec3::sZero();
			normal.SetComponent(a, 1.0f);
		}
	}
	return normal;
}

void JoltCustomVoxelGridShape::GetSupportingFace(const JPH::SubShapeID &p_sub_shape_id, JPH::Vec3Arg p_direction, JPH::Vec3Arg p_scale, JPH::Mat44Arg p_center_of_mass_transform, SupportingFace &p_vertices) const {
	int cell, box;
	decode(p_sub_shape_id, cell, box);
	const JPH::AABox scaled = box_bounds(cell, box).Scaled(p_scale);
	scaled.GetSupportingFace(p_direction, p_vertices);
	for (JPH::Vec3 &vertex : p_vertices) {
		vertex = p_center_of_mass_transform * vertex;
	}
}

void JoltCustomVoxelGridShape::GetSubmergedVolume(JPH::Mat44Arg p_center_of_mass_transform, JPH::Vec3Arg p_scale, const JPH::Plane &p_surface, float &p_total_volume, float &p_submerged_volume, JPH::Vec3 &p_center_of_buoyancy
#ifdef JPH_DEBUG_RENDERER
		,
		JPH::RVec3Arg p_base_offset
#endif
) const {
	p_total_volume = 0.0f;
	p_submerged_volume = 0.0f;
	p_center_of_buoyancy = JPH::Vec3::sZero();
}

// Walks the cells the ray crosses, nearest first (Amanatides and Woo), calling visit(cell, module,
// enter, exit) until it returns false; a ray along a cell boundary walks the columns on both sides.
template <class F>
void JoltCustomVoxelGridShape::walk_ray(const JPH::RayCast &p_ray, F &&p_visit) const {
	const JPH::RayInvDirection inverse(p_ray.mDirection);
	float tmin, tmax;
	JPH::RayAABox(p_ray.mOrigin, inverse, bounds.mMin, bounds.mMax, tmin, tmax);
	if (tmin > tmax || tmax < 0.0f || tmin > 1.0f) {
		return;
	}
	tmin = std::max(tmin, 0.0f);
	tmax = std::min(tmax, 1.0f);
	const JPH::Vec3 start = (p_ray.mOrigin + p_ray.mDirection * tmin - data.origin) / data.cell_meters;
	// A ray along a cell boundary (no motion on that axis) touches the cells on both sides of it: a
	// box face lying on the boundary belongs to the other cell, so both columns are walked.
	int low[3], high[3];
	for (int a = 0; a < 3; a++) {
		const float c = start[a];
		const bool along = p_ray.mDirection[a] == 0.0f;
		low[a] = std::clamp(int(std::floor(along ? c - 1.0e-4f : c)), 0, data.size[a] - 1);
		high[a] = std::clamp(int(std::floor(along ? c + 1.0e-4f : c)), 0, data.size[a] - 1);
	}
	for (int ci = low[0]; ci <= high[0]; ci++) {
		for (int cj = low[1]; cj <= high[1]; cj++) {
			for (int ck = low[2]; ck <= high[2]; ck++) {
				int cell[3] = { ci, cj, ck }, step[3];
				float next[3], delta[3];
				for (int a = 0; a < 3; a++) {
					const float d = p_ray.mDirection[a];
					if (d > 0) {
						step[a] = 1;
						delta[a] = data.cell_meters / d;
						next[a] = (data.origin[a] + (cell[a] + 1) * data.cell_meters - p_ray.mOrigin[a]) / d;
					} else if (d < 0) {
						step[a] = -1;
						delta[a] = -data.cell_meters / d;
						next[a] = (data.origin[a] + cell[a] * data.cell_meters - p_ray.mOrigin[a]) / d;
					} else {
						step[a] = 0;
						delta[a] = FLT_MAX;
						next[a] = FLT_MAX;
					}
				}
				float enter = tmin;
				for (;;) {
					const int a = next[0] < next[1] ? (next[0] < next[2] ? 0 : 2) : (next[1] < next[2] ? 1 : 2);
					const float exit = std::min(next[a], tmax);
					const int index = data.padded_modules[padded_index(cell[0], cell[1], cell[2])];
					if (index >= 0 && !p_visit(cell_index(cell[0], cell[1], cell[2]), index, enter, exit)) {
						break;
					}
					if (next[a] > tmax) {
						break;
					}
					cell[a] += step[a];
					if (cell[a] < 0 || cell[a] >= data.size[a]) {
						break;
					}
					enter = next[a];
					next[a] += delta[a];
				}
			}
		}
	}
}

namespace {

// The boxes of a cell the ray enters, four at a time: visit(box, fraction) for each box it meets at a
// fraction below the limit (negative when it starts inside the box).
template <class F>
void ray_boxes(const JoltCustomVoxelGridShape::Module &p_module, JPH::Vec3Arg p_local_origin, const JPH::RayInvDirection &p_inverse, float p_limit, F &&p_visit) {
	const float *groups = p_module.groups.data();
	const JPH::Vec4 limit = JPH::Vec4::sReplicate(p_limit);
	for (size_t g = 0; g < p_module.groups.size(); g += 24) {
		const float *group = groups + g;
		const JPH::Vec4 t = JPH::RayAABox4(p_local_origin, p_inverse, JPH::Vec4::sLoadFloat4((const JPH::Float4 *)group), JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 4)), JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 8)),
				JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 12)), JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 16)), JPH::Vec4::sLoadFloat4((const JPH::Float4 *)(group + 20)));
		int hits = JPH::Vec4::sLess(t, limit).GetTrues();
		while (hits != 0) {
			const int lane = JPH::CountTrailingZeros(JPH::uint32(hits));
			hits &= hits - 1;
			p_visit(int(g / 24) * 4 + lane, t[lane]);
		}
	}
}

} // namespace

namespace {

// The face a ray enters a box through (the slab it enters last), or, leaving it, the face it leaves
// through (the slab it leaves first); faces as JoltCustomVoxelGridShape's sub-shape IDs number them.
int ray_face(const JPH::AABox &p_box, const JPH::RayCast &p_ray, bool p_leaving) {
	int face = 2;
	float best = p_leaving ? FLT_MAX : -FLT_MAX;
	for (int a = 0; a < 3; a++) {
		const float d = p_ray.mDirection[a];
		if (d == 0.0f) {
			continue;
		}
		const bool towards_min = (d > 0.0f) != p_leaving;
		const float t = ((towards_min ? p_box.mMin[a] : p_box.mMax[a]) - p_ray.mOrigin[a]) / d;
		if (p_leaving ? t < best : t > best) {
			best = t;
			face = a * 2 + (towards_min ? 0 : 1);
		}
	}
	return face;
}

} // namespace

bool JoltCustomVoxelGridShape::CastRay(const JPH::RayCast &p_ray, const JPH::SubShapeIDCreator &p_sub_shape_id_creator, JPH::RayCastResult &p_hit) const {
	const JPH::RayInvDirection inverse(p_ray.mDirection);
	bool found = false;
	walk_ray(p_ray, [&](int p_cell, int p_module, float p_enter, float p_exit) {
		const Module &module = *data.modules[p_module];
		int i, j, k;
		cell_coordinates(p_cell, i, j, k);
		const JPH::Vec3 corner = data.origin + JPH::Vec3(float(i), float(j), float(k)) * data.cell_meters;
		ray_boxes(module, p_ray.mOrigin - corner, inverse, p_hit.mFraction, [&](int p_box, float p_fraction) {
			if (p_fraction < 0.0f) {
				return;
			}
			p_hit.mFraction = p_fraction;
			p_hit.mSubShapeID2 = encode(p_sub_shape_id_creator, p_cell, p_box, ray_face(box_bounds(p_cell, p_box), p_ray, false));
			found = true;
		});
		return !(found && p_hit.mFraction <= p_exit);
	});
	return found;
}

void JoltCustomVoxelGridShape::CastRay(const JPH::RayCast &p_ray, const JPH::RayCastSettings &p_ray_cast_settings, const JPH::SubShapeIDCreator &p_sub_shape_id_creator, JPH::CastRayCollector &p_collector, const JPH::ShapeFilter &p_shape_filter) const {
	if (!p_shape_filter.ShouldCollide(this, p_sub_shape_id_creator.GetID())) {
		return;
	}
	const JPH::RayInvDirection inverse(p_ray.mDirection);
	walk_ray(p_ray, [&](int p_cell, int p_module, float p_enter, float p_exit) {
		const Module &module = *data.modules[p_module];
		int i, j, k;
		cell_coordinates(p_cell, i, j, k);
		const JPH::Vec3 corner = data.origin + JPH::Vec3(float(i), float(j), float(k)) * data.cell_meters;
		bool stop = false;
		ray_boxes(module, p_ray.mOrigin - corner, inverse, std::min(1.0f, p_collector.GetEarlyOutFraction()), [&](int p_box, float p_fraction) {
			if (stop) {
				return;
			}
			int face = UNKNOWN_FACE;
			if (p_fraction < 0.0f) {
				// Inside the box: a solid hit at the start, or the back face where it leaves.
				if (p_ray_cast_settings.mTreatConvexAsSolid) {
					p_fraction = 0.0f;
				} else if (p_ray_cast_settings.mBackFaceModeTriangles == JPH::EBackFaceMode::CollideWithBackFaces) {
					const JPH::AABox box = box_bounds(p_cell, p_box);
					float low, high;
					JPH::RayAABox(p_ray.mOrigin, inverse, box.mMin, box.mMax, low, high);
					p_fraction = high;
					face = ray_face(box, p_ray, true);
				} else {
					return;
				}
			} else {
				face = ray_face(box_bounds(p_cell, p_box), p_ray, false);
			}
			if (p_fraction > 1.0f || p_fraction >= p_collector.GetEarlyOutFraction()) {
				return;
			}
			JPH::RayCastResult hit;
			hit.mBodyID = JPH::TransformedShape::sGetBodyID(p_collector.GetContext());
			hit.mFraction = p_fraction;
			hit.mSubShapeID2 = encode(p_sub_shape_id_creator, p_cell, p_box, face);
			p_collector.AddHit(hit);
			if (p_collector.ShouldEarlyOut()) {
				stop = true;
			}
		});
		return !stop && p_collector.GetEarlyOutFraction() > p_exit;
	});
}

void JoltCustomVoxelGridShape::CollidePoint(JPH::Vec3Arg p_point, const JPH::SubShapeIDCreator &p_sub_shape_id_creator, JPH::CollidePointCollector &p_collector, const JPH::ShapeFilter &p_shape_filter) const {
	if (!p_shape_filter.ShouldCollide(this, p_sub_shape_id_creator.GetID())) {
		return;
	}
	for_boxes(JPH::AABox(p_point, p_point), [&](int p_cell, int p_box, JPH::Vec3 p_low, JPH::Vec3 p_high) {
		JPH::CollidePointResult result;
		result.mBodyID = JPH::TransformedShape::sGetBodyID(p_collector.GetContext());
		result.mSubShapeID2 = encode(p_sub_shape_id_creator, p_cell, p_box, UNKNOWN_FACE);
		p_collector.AddHit(result);
		return !p_collector.ShouldEarlyOut();
	});
}

void JoltCustomVoxelGridShape::collide_convex_vs_grid(const JPH::Shape *p_shape1, const JPH::Shape *p_shape2, JPH::Vec3Arg p_scale1, JPH::Vec3Arg p_scale2, JPH::Mat44Arg p_center_of_mass_transform1, JPH::Mat44Arg p_center_of_mass_transform2, const JPH::SubShapeIDCreator &p_sub_shape_id_creator1, const JPH::SubShapeIDCreator &p_sub_shape_id_creator2, const JPH::CollideShapeSettings &p_collide_shape_settings, JPH::CollideShapeCollector &p_collector, const JPH::ShapeFilter &p_shape_filter) {
	const JoltCustomVoxelGridShape *grid = static_cast<const JoltCustomVoxelGridShape *>(p_shape2);
	const JPH::Mat44 into = p_center_of_mass_transform2.InversedRotationTranslation() * p_center_of_mass_transform1;
	JPH::AABox other_bounds = p_shape1->GetLocalBounds().Scaled(p_scale1).Transformed(into);
	other_bounds.ExpandBy(JPH::Vec3::sReplicate(p_collide_shape_settings.mMaxSeparationDistance));
	JPH::ConvexShape::SupportBuffer buffer;
	const JPH::ConvexShape::Support *support = static_cast<const JPH::ConvexShape *>(p_shape1)->GetSupportFunction(JPH::ConvexShape::ESupportMode::IncludeConvexRadius, buffer, p_scale1);
	JoltVoxelGridCoveredFaceCollector covered(p_collector, p_center_of_mass_transform2, *grid, *support, into, p_collide_shape_settings.mMaxSeparationDistance);
	grid->for_boxes(other_bounds, [&](int p_cell, int p_box, JPH::Vec3 p_low, JPH::Vec3 p_high) {
		const JPH::Vec3 half = 0.5f * (p_high - p_low);
		JPH::BoxShape box(half, 0.0f);
		box.SetEmbedded();
		covered.box(p_cell, p_box);
		JPH::CollisionDispatch::sCollideShapeVsShape(p_shape1, &box, p_scale1, JPH::Vec3::sOne(), p_center_of_mass_transform1, p_center_of_mass_transform2 * JPH::Mat44::sTranslation(0.5f * (p_low + p_high)),
				p_sub_shape_id_creator1, p_sub_shape_id_creator2.PushID(p_cell, grid->cell_bits).PushID(p_box, grid->box_bits).PushID(UNKNOWN_FACE, FACE_BITS), p_collide_shape_settings, covered, p_shape_filter);
		return !p_collector.ShouldEarlyOut();
	});
}

void JoltCustomVoxelGridShape::cast_convex_vs_grid(const JPH::ShapeCast &p_shape_cast, const JPH::ShapeCastSettings &p_shape_cast_settings, const JPH::Shape *p_shape, JPH::Vec3Arg p_scale, const JPH::ShapeFilter &p_shape_filter, JPH::Mat44Arg p_center_of_mass_transform2, const JPH::SubShapeIDCreator &p_sub_shape_id_creator1, const JPH::SubShapeIDCreator &p_sub_shape_id_creator2, JPH::CastShapeCollector &p_collector) {
	// Godot's motion queries collide a swept shape instead; casts are answered box by box, with the
	// covered faces not filtered.
	const JoltCustomVoxelGridShape *grid = static_cast<const JoltCustomVoxelGridShape *>(p_shape);
	JPH::AABox swept = p_shape_cast.mShapeWorldBounds;
	JPH::AABox moved = swept;
	moved.Translate(p_shape_cast.mDirection);
	swept.Encapsulate(moved);
	grid->for_boxes(swept, [&](int p_cell, int p_box, JPH::Vec3 p_low, JPH::Vec3 p_high) {
		const JPH::Vec3 half = 0.5f * (p_high - p_low), centre = 0.5f * (p_low + p_high);
		JPH::BoxShape box(half, 0.0f);
		box.SetEmbedded();
		const JPH::ShapeCast local = p_shape_cast.PostTransformed(JPH::Mat44::sTranslation(-centre));
		JPH::CollisionDispatch::sCastShapeVsShapeLocalSpace(local, p_shape_cast_settings, &box, JPH::Vec3::sOne(), p_shape_filter, p_center_of_mass_transform2 * JPH::Mat44::sTranslation(centre),
				p_sub_shape_id_creator1, p_sub_shape_id_creator2.PushID(p_cell, grid->cell_bits).PushID(p_box, grid->box_bits).PushID(UNKNOWN_FACE, FACE_BITS), p_collector);
		return !p_collector.ShouldEarlyOut();
	});
}

JPH::ShapeRefC JoltCustomVoxelGridShape::make_box_compound() const {
	JPH::StaticCompoundShapeSettings settings;
	for_boxes(bounds, [&](int p_cell, int p_box, JPH::Vec3 p_low, JPH::Vec3 p_high) {
		settings.AddShape(0.5f * (p_low + p_high), JPH::Quat::sIdentity(), new JPH::BoxShape(0.5f * (p_high - p_low), 0.0f));
		return true;
	});
	if (settings.mSubShapes.empty()) {
		return new JPH::SphereShape(0.1f);
	}
	const JPH::ShapeSettings::ShapeResult result = settings.Create();
	return result.HasError() ? JPH::ShapeRefC(new JPH::SphereShape(0.1f)) : result.Get();
}

void JoltCustomVoxelGridShape::register_type() {
	JPH::ShapeFunctions &shape_functions = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::VOXEL_GRID);
	shape_functions.mConstruct = construct_voxel_grid;
	shape_functions.mColor = JPH::Color::sOrange;
	for (const JPH::EShapeSubType sub_type : JPH::sConvexSubShapeTypes) {
		// Godot's ray shape collides through its own function, registered with it.
		if (sub_type == JoltCustomShapeSubType::RAY) {
			continue;
		}
		JPH::CollisionDispatch::sRegisterCollideShape(sub_type, JoltCustomShapeSubType::VOXEL_GRID, collide_convex_vs_grid);
		JPH::CollisionDispatch::sRegisterCastShape(sub_type, JoltCustomShapeSubType::VOXEL_GRID, cast_convex_vs_grid);
		JPH::CollisionDispatch::sRegisterCollideShape(JoltCustomShapeSubType::VOXEL_GRID, sub_type, JPH::CollisionDispatch::sReversedCollideShape);
		JPH::CollisionDispatch::sRegisterCastShape(JoltCustomShapeSubType::VOXEL_GRID, sub_type, JPH::CollisionDispatch::sReversedCastShape);
	}
}
