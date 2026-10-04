/**************************************************************************/
/*  jolt_voxel_grid_shape_3d.cpp                                          */
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

#include "jolt_voxel_grid_shape_3d.h"

#include "jolt_custom_voxel_grid_shape.h"

JPH::ShapeRefC JoltVoxelGridShape3D::_build() const {
	if (data.is_empty()) {
		return nullptr;
	}
	const Vector3i size = data["size"];
	const Vector3 origin = data["origin"];
	const PackedInt32Array cells = data["cells"];
	const PackedInt32Array box_offsets = data["box_offsets"];
	const PackedFloat32Array boxes = data["boxes"];

	JoltCustomVoxelGridShape::Data grid;
	grid.cell_meters = data["cell_size"];
	grid.cell_voxels = data["cell_voxels"];
	grid.size[0] = size.x;
	grid.size[1] = size.y;
	grid.size[2] = size.z;
	grid.origin = JPH::Vec3((float)origin.x, (float)origin.y, (float)origin.z);
	grid.padded_modules.assign(cells.ptr(), cells.ptr() + cells.size());
	const int module_count = box_offsets.size() - 1;
	grid.modules.reserve((size_t)module_count);
	for (int m = 0; m < module_count; m++) {
		const int first = box_offsets[m], last = box_offsets[m + 1];
		grid.modules.push_back(JoltCustomVoxelGridShape::get_module(boxes.ptr() + (size_t)first * 6, last - first, grid.cell_meters, grid.cell_voxels));
	}
	return new JoltCustomVoxelGridShape(std::move(grid));
}

void JoltVoxelGridShape3D::set_data(const Variant &p_data) {
	ERR_FAIL_COND_MSG(p_data.get_type() != Variant::DICTIONARY, "A voxel grid shape takes a Dictionary.");
	const Dictionary new_data = p_data;
	ERR_FAIL_COND(new_data.get("cell_size", Variant()).get_type() != Variant::FLOAT);
	ERR_FAIL_COND(new_data.get("cell_voxels", Variant()).get_type() != Variant::INT);
	ERR_FAIL_COND(new_data.get("size", Variant()).get_type() != Variant::VECTOR3I);
	ERR_FAIL_COND(new_data.get("origin", Variant()).get_type() != Variant::VECTOR3);
	ERR_FAIL_COND(new_data.get("cells", Variant()).get_type() != Variant::PACKED_INT32_ARRAY);
	ERR_FAIL_COND(new_data.get("box_offsets", Variant()).get_type() != Variant::PACKED_INT32_ARRAY);
	ERR_FAIL_COND(new_data.get("boxes", Variant()).get_type() != Variant::PACKED_FLOAT32_ARRAY);

	const float cell_size = new_data["cell_size"];
	const int cell_voxels = new_data["cell_voxels"];
	const Vector3i size = new_data["size"];
	const PackedInt32Array cells = new_data["cells"];
	const PackedInt32Array box_offsets = new_data["box_offsets"];
	const PackedFloat32Array boxes = new_data["boxes"];
	ERR_FAIL_COND_MSG(cell_size <= 0.0f || cell_voxels < 1 || cell_voxels > 32, "A voxel grid shape needs a positive cell size and 1 to 32 voxels a cell side.");
	ERR_FAIL_COND_MSG(size.x < 1 || size.y < 1 || size.z < 1, "A voxel grid shape needs at least one cell.");
	ERR_FAIL_COND_MSG(cells.size() != (size.x + 2) * (size.y + 2) * (size.z + 2), "A voxel grid shape's cells must cover the grid and its one-cell ring.");
	ERR_FAIL_COND_MSG(box_offsets.is_empty() || box_offsets[0] != 0 || box_offsets[box_offsets.size() - 1] * 6 != boxes.size(), "A voxel grid shape's box offsets must start at 0 and end at the box count.");
	const int module_count = box_offsets.size() - 1;
	for (int m = 0; m < module_count; m++) {
		ERR_FAIL_COND_MSG(box_offsets[m + 1] < box_offsets[m], "A voxel grid shape's box offsets must not decrease.");
	}
	for (int i = 0; i < cells.size(); i++) {
		ERR_FAIL_COND_MSG(cells[i] < -1 || cells[i] >= module_count, "A voxel grid shape's cell names a module it does not have.");
	}

	data = new_data;
	const Vector3 origin = data["origin"];
	aabb = AABB(origin, Vector3(size) * cell_size);
	destroy();
}

String JoltVoxelGridShape3D::to_string() const {
	const Vector3i size = data.get("size", Vector3i());
	return vformat("{size=%s}", size);
}
