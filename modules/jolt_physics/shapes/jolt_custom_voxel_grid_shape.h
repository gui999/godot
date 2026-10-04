/**************************************************************************/
/*  jolt_custom_voxel_grid_shape.h                                        */
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

#pragma once

// Terra Prime (M19.9.8, D221): a voxel box-grid shape. A grid of cells, each naming a module whose
// solid axis-aligned boxes are the collision; there is no tree and no triangle. Queries walk the cells
// their bounds overlap and collide with each box through Jolt's own convex-vs-box code. A box face that
// solid voxels cover whole (in its own cell, or in a neighbour cell, the grid's one-cell ring included)
// cannot be touched from outside, so a contact that would push through it is answered on the box's
// exposed faces instead (the least penetrated one, by the separating-axis test), never dropped while
// the shapes overlap: the voxel counterpart of a mesh's active edges. The shape is immutable once built.

#include "jolt_custom_shape_type.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <cstdint>
#include <vector>

class JoltCustomVoxelGridShape final : public JPH::Shape {
public:
	// A module's boxes and what is derived from them for queries, shared by every row that places it
	// (a cache keyed by the boxes, so deriving happens once a module, not once a row).
	struct Module : public JPH::RefTarget<Module> {
		std::vector<float> source; // the boxes as given: min xyz, max xyz in metres, cell frame
		// The boxes in metres in the cell's frame, four at a time for SIMD tests: per group min x[4],
		// y[4], z[4], max x[4], y[4], z[4]; padding lanes are inverted (never hit, never overlap).
		std::vector<float> groups;
		int box_count = 0;
		// Per box, the faces its own cell's voxels cover whole, and the faces on the cell's boundary
		// (their cover depends on the neighbour). Faces: 0 -X, 1 +X, 2 -Y, 3 +Y, 4 -Z, 5 +Z.
		std::vector<uint8_t> covered_faces;
		std::vector<uint8_t> boundary_faces;
		// Per side, the solid voxels of the boundary layer, one row of u bits per v (u = axis + 1,
		// v = axis + 2, modulo 3). Empty when the module's boxes do not lie on the voxel grid.
		std::vector<uint32_t> boundary_layers;
		// Per box, its integer voxel bounds (low xyz, size xyz), when on the voxel grid.
		std::vector<int> voxel_boxes;
	};

	struct Data {
		float cell_meters = 2.0f;
		int cell_voxels = 20;
		// The collidable cells (size) and their corner in the shape's frame.
		int size[3] = { 0, 0, 0 };
		JPH::Vec3 origin = JPH::Vec3::sZero();
		// Per cell of the grid padded with a one-cell ring on every side: a module index or -1. Ring
		// cells are read only for face cover.
		std::vector<int32_t> padded_modules;
		std::vector<JPH::RefConst<Module>> modules;
	};

	// A module's query data for its boxes (min xyz, max xyz in metres, cell frame), from the cache or
	// derived now. Thread-safe.
	static JPH::RefConst<Module> get_module(const float *p_boxes, int p_box_count, float p_cell_meters, int p_cell_voxels);

	static void register_type();

	explicit JoltCustomVoxelGridShape(Data &&p_data);

	bool MustBeStatic() const override { return true; }
	JPH::AABox GetLocalBounds() const override { return bounds; }
	JPH::uint GetSubShapeIDBitsRecursive() const override { return cell_bits + box_bits + FACE_BITS; }
	float GetInnerRadius() const override { return 0.0f; }
	JPH::MassProperties GetMassProperties() const override { return JPH::MassProperties(); }
	const JPH::PhysicsMaterial *GetMaterial(const JPH::SubShapeID &p_sub_shape_id) const override;
	JPH::Vec3 GetSurfaceNormal(const JPH::SubShapeID &p_sub_shape_id, JPH::Vec3Arg p_local_surface_position) const override;
	void GetSupportingFace(const JPH::SubShapeID &p_sub_shape_id, JPH::Vec3Arg p_direction, JPH::Vec3Arg p_scale, JPH::Mat44Arg p_center_of_mass_transform, SupportingFace &p_vertices) const override;
	void GetSubmergedVolume(JPH::Mat44Arg p_center_of_mass_transform, JPH::Vec3Arg p_scale, const JPH::Plane &p_surface, float &p_total_volume, float &p_submerged_volume, JPH::Vec3 &p_center_of_buoyancy
#ifdef JPH_DEBUG_RENDERER
			,
			JPH::RVec3Arg p_base_offset
#endif
	) const override;
#ifdef JPH_DEBUG_RENDERER
	void Draw(JPH::DebugRenderer *p_renderer, JPH::RMat44Arg p_center_of_mass_transform, JPH::Vec3Arg p_scale, JPH::ColorArg p_color, bool p_use_material_colors, bool p_draw_wireframe) const override {}
#endif
	bool CastRay(const JPH::RayCast &p_ray, const JPH::SubShapeIDCreator &p_sub_shape_id_creator, JPH::RayCastResult &p_hit) const override;
	void CastRay(const JPH::RayCast &p_ray, const JPH::RayCastSettings &p_ray_cast_settings, const JPH::SubShapeIDCreator &p_sub_shape_id_creator, JPH::CastRayCollector &p_collector, const JPH::ShapeFilter &p_shape_filter = {}) const override;
	void CollidePoint(JPH::Vec3Arg p_point, const JPH::SubShapeIDCreator &p_sub_shape_id_creator, JPH::CollidePointCollector &p_collector, const JPH::ShapeFilter &p_shape_filter = {}) const override;
	void CollideSoftBodyVertices(JPH::Mat44Arg p_center_of_mass_transform, JPH::Vec3Arg p_scale, const JPH::CollideSoftBodyVertexIterator &p_vertices, JPH::uint p_num_vertices, int p_colliding_shape_index) const override {}
	void GetTrianglesStart(GetTrianglesContext &p_context, const JPH::AABox &p_box, JPH::Vec3Arg p_position_com, JPH::QuatArg p_rotation, JPH::Vec3Arg p_scale) const override {}
	int GetTrianglesNext(GetTrianglesContext &p_context, int p_max_triangles_requested, JPH::Float3 *p_triangle_vertices, const JPH::PhysicsMaterial **p_materials = nullptr) const override { return 0; }
	Stats GetStats() const override;
	float GetVolume() const override { return 0.0f; }

	// The boxes as a static compound of box shapes, for snapshots that cannot hold a custom shape.
	JPH::ShapeRefC make_box_compound() const;

	int get_box_count() const;

private:
	// A sub-shape ID names the cell, the box and the face a ray entered through (0 -X, 1 +X, 2 -Y,
	// 3 +Y, 4 -Z, 5 +Z), or UNKNOWN_FACE for a contact, so a ray's normal is the face it hit, also
	// where the hit lies on a box's edge.
	static constexpr JPH::uint FACE_BITS = 3;
	static constexpr int UNKNOWN_FACE = 6;

	int padded_index(int p_i, int p_j, int p_k) const { return ((p_j + 1) * (data.size[2] + 2) + (p_k + 1)) * (data.size[0] + 2) + (p_i + 1); }
	int cell_index(int p_i, int p_j, int p_k) const { return (p_j * data.size[2] + p_k) * data.size[0] + p_i; }
	void cell_coordinates(int p_cell, int &r_i, int &r_j, int &r_k) const;
	int module_at(int p_cell) const;
	void decode(const JPH::SubShapeID &p_sub_shape_id, int &r_cell, int &r_box) const;
	void decode(const JPH::SubShapeID &p_sub_shape_id, int &r_cell, int &r_box, int &r_face) const;
	JPH::SubShapeID encode(const JPH::SubShapeIDCreator &p_creator, int p_cell, int p_box, int p_face) const { return p_creator.PushID(p_cell, cell_bits).PushID(p_box, box_bits).PushID(p_face, FACE_BITS).GetID(); }
	JPH::AABox box_bounds(int p_cell, int p_box) const;
	uint8_t covered_faces(int p_cell, int p_box) const;

	template <class F>
	void for_boxes(const JPH::AABox &p_bounds, F &&p_visit) const;
	template <class F>
	void walk_ray(const JPH::RayCast &p_ray, F &&p_visit) const;

	static void collide_convex_vs_grid(const JPH::Shape *p_shape1, const JPH::Shape *p_shape2, JPH::Vec3Arg p_scale1, JPH::Vec3Arg p_scale2, JPH::Mat44Arg p_center_of_mass_transform1, JPH::Mat44Arg p_center_of_mass_transform2, const JPH::SubShapeIDCreator &p_sub_shape_id_creator1, const JPH::SubShapeIDCreator &p_sub_shape_id_creator2, const JPH::CollideShapeSettings &p_collide_shape_settings, JPH::CollideShapeCollector &p_collector, const JPH::ShapeFilter &p_shape_filter);
	static void cast_convex_vs_grid(const JPH::ShapeCast &p_shape_cast, const JPH::ShapeCastSettings &p_shape_cast_settings, const JPH::Shape *p_shape, JPH::Vec3Arg p_scale, const JPH::ShapeFilter &p_shape_filter, JPH::Mat44Arg p_center_of_mass_transform2, const JPH::SubShapeIDCreator &p_sub_shape_id_creator1, const JPH::SubShapeIDCreator &p_sub_shape_id_creator2, JPH::CastShapeCollector &p_collector);

	friend class JoltVoxelGridCoveredFaceCollector;

	Data data;
	JPH::uint cell_bits = 0;
	JPH::uint box_bits = 0;
	JPH::AABox bounds;
};
