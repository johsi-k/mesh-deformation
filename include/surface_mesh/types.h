//=============================================================================
// Copyright (C) 2013 Graphics & Geometry Group, Bielefeld University
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public License
// as published by the Free Software Foundation, version 2.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU Library General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
//=============================================================================


#ifndef SURFACE_MESH_TYPES_H
#define SURFACE_MESH_TYPES_H


//== INCLUDES =================================================================


#include <Eigen/Dense>


//=============================================================================


namespace surface_mesh {

typedef double Scalar;

//=============================================================================
typedef Eigen::Vector3i Indices; ///< vertex indices of a face

typedef Eigen::Matrix<Scalar, 2, 1> Vec2; ///< 2D vector type
typedef Eigen::Matrix<Scalar, 3, 1> Vec3; ///< 3D vector type

typedef Vec3 Point;              ///< Point type
typedef Vec3 Normal;             ///< Normal type
typedef Vec3 Color;              ///< Color type
typedef Vec2 Texture_coordinate; ///< Texture coordinate type

inline Scalar nan(){ return std::numeric_limits<Scalar>::quiet_NaN(); }
inline Scalar inf(){ return std::numeric_limits<Scalar>::max(); } 

//=============================================================================
} // namespace surface_mesh
//=============================================================================
#endif // SURFACE_MESH_TYPES_H
//============================================================================
