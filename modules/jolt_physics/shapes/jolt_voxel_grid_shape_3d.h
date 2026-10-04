/**************************************************************************/
/*  jolt_voxel_grid_shape_3d.h                                            */
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

// Terra Prime (M19.9.8, D221): Godot's custom shape (PhysicsServer3D::custom_shape_create) as a voxel
// box-grid shape. Its data is a Dictionary: "cell_size" (float, metres), "cell_voxels" (int, voxels a
// cell side), "size" (Vector3i, the collidable cells), "origin" (Vector3, the first cell's corner in the
// shape's frame), "cells" (PackedInt32Array, a module index or -1 per cell of the grid padded with a
// one-cell ring, x fastest, then z, then y), "box_offsets" (PackedInt32Array, each module's first box
// and one past the last) and "boxes" (PackedFloat32Array, per box min xyz and max xyz in metres in the
// cell's frame). Ring cells are read only for face cover.

#include "jolt_shape_3d.h"

class JoltVoxelGridShape3D final : public JoltShape3D {
	AABB aabb;
	Dictionary data;

	virtual JPH::ShapeRefC _build() const override;

public:
	virtual ShapeType get_type() const override { return ShapeType::SHAPE_CUSTOM; }
	virtual bool is_convex() const override { return false; }

	virtual Variant get_data() const override { return data; }
	virtual void set_data(const Variant &p_data) override;

	virtual float get_margin() const override { return 0.0f; }
	virtual void set_margin(float p_margin) override {}

	virtual AABB get_aabb() const override { return aabb; }

	String to_string() const;
};
