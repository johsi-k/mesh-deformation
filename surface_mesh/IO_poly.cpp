//=============================================================================
// Copyright (C) 2001-2005 by Computer Graphics Group, RWTH Aachen
// Copyright (C) 2011-2013 by Graphics & Geometry Group, Bielefeld University
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


//== INCLUDES =================================================================


#include <surface_mesh/IO.h>
#include <surface_mesh/tinyply.h>

#include <cstdio>
#include <fstream>
#include <iostream>

//== NAMESPACES ===============================================================


namespace surface_mesh {


//== IMPLEMENTATION ===========================================================


// helper function
template <typename T> size_t read_poly_helper(FILE* in, T& t)
{
    return fread((char*)&t, 1, sizeof(t), in);
}


//-----------------------------------------------------------------------------

bool read_poly(Surface_mesh& mesh, const std::string& filename)
{
    // Read the file and create a std::istringstream suitable
    // for the lib -- tinyply does not perform any file i/o.
    std::ifstream ss(filename, std::ios::binary);

    // Parse the ASCII header fields
    tinyply::PlyFile file(ss);

    std::vector<float> verts;
    std::vector<float> norms;
    std::vector<uint8_t> colors;

    std::vector<uint32_t> faces;

    uint32_t vertexCount, normalCount, colorCount, faceCount;
    vertexCount = normalCount = colorCount = faceCount = 0;


    // The count returns the number of instances of the property group. The vectors
    // above will be resized into a multiple of the property group size as
    // they are "flattened"... i.e. verts = {x, y, z, x, y, z, ...}
    vertexCount = file.request_properties_from_element("vertex", {"x", "y", "z"}, verts);
    normalCount = file.request_properties_from_element("vertex", {"nx", "ny", "nz"}, norms);
    colorCount = file.request_properties_from_element("vertex", {"red", "green", "blue"}, colors);

    // For properties that are list types, it is possibly to specify the expected count (ideal if a
    // consumer of this library knows the layout of their format a-priori). Otherwise, tinyply
    // defers allocation of memory until the first instance of the property has been found
    // as implemented in file.read(ss)
    faceCount = file.request_properties_from_element("face", {"vertex_indices"}, faces, 3);

    file.read(ss);

    // clear mesh
    mesh.clear();

    Surface_mesh::Vertex_property<Normal> normals_prop = mesh.vertex_property<Normal>("v:normal");
    Surface_mesh::Vertex_property<Color>  colors_prop = mesh.vertex_property<Color>("v:color");


    std::cout << vertexCount << std::endl;
    std::cout << normalCount << std::endl;
    std::cout << colorCount << std::endl;

    for(unsigned int i=0; i<vertexCount; i++) {
        Surface_mesh::Vertex v = mesh.add_vertex(Point(verts[i*3],verts[i*3+1],verts[i*3+2]));
        if(i < normalCount)
            normals_prop[v] = Normal(norms[i*3],norms[i*3+1],norms[i*3+2]);
        if(i < colorCount)
            colors_prop[v] = Color(colors[i*3]/255.,colors[i*3+1]/255.,colors[i*3+2]/255.);
    }

    for(unsigned int i=0; i<faceCount; i++) {
        Surface_mesh::Vertex v0(faces[i*3]);
        Surface_mesh::Vertex v1(faces[i*3+1]);
        Surface_mesh::Vertex v2(faces[i*3+2]);
        mesh.add_triangle(v0,v1,v2);
    }

    return true;
}


//-----------------------------------------------------------------------------


bool write_poly(const Surface_mesh& mesh, const std::string& filename)
{
    FILE* out = fopen(filename.c_str(), "w");
    if (!out)
        return false;

    bool  has_normals   = false;
    bool  has_colors = false;
    Surface_mesh::Vertex_property<Normal> normals = mesh.get_vertex_property<Normal>("v:normal");
    Surface_mesh::Vertex_property<Color>  colors = mesh.get_vertex_property<Color>("v:color");
    if (normals)   has_normals = true;
    if (colors) has_colors = true;

    // header
    fprintf(out, "ply\n");
    fprintf(out, "format ascii 1.0\n");
    fprintf(out, "element vertex %d\n", mesh.n_vertices());
    fprintf(out, "property float x\n");
    fprintf(out, "property float y\n");
    fprintf(out, "property float z\n");
    if(has_normals) {
        fprintf(out, "property float nx\n");
        fprintf(out, "property float ny\n");
        fprintf(out, "property float nz\n");
    }
    if(has_colors) {
        fprintf(out, "property uchar red \n");
        fprintf(out, "property uchar green\n");
        fprintf(out, "property uchar blue\n");
    }
    fprintf(out, "element face %d\n", mesh.n_faces());
    fprintf(out, "property list uchar uint vertex_indices\n");
    fprintf(out, "end_header\n");

    Surface_mesh::Vertex_property<Point> points = mesh.get_vertex_property<Point>("v:point");
    for (Surface_mesh::Vertex_iterator vit=mesh.vertices_begin(); vit!=mesh.vertices_end(); ++vit)
    {
        const Point& p = points[*vit];
        fprintf(out, "%.10f %.10f %.10f", p[0], p[1], p[2]);

        if (has_normals)
        {
            const Normal& n = normals[*vit];
            fprintf(out, " %.10f %.10f %.10f", n[0], n[1], n[2]);
        }

        if (has_colors)
        {
            Eigen::Vector3i c = (colors[*vit]*255).cast<int>();
            fprintf(out, " %d %d %d", c[0], c[1], c[2]);
        }

        fprintf(out, "\n");
    }

    // faces
    for (Surface_mesh::Face_iterator fit=mesh.faces_begin(); fit!=mesh.faces_end(); ++fit)
    {
        int nV = mesh.valence(*fit);
        fprintf(out, "%d", nV);
        Surface_mesh::Vertex_around_face_circulator fvit=mesh.vertices(*fit), fvend=fvit;
        do
        {
            fprintf(out, " %d", (*fvit).idx());
        }
        while (++fvit != fvend);
        fprintf(out, "\n");
    }

    fclose(out);
    return true;
}


//=============================================================================
} // namespace surface_mesh
//=============================================================================
