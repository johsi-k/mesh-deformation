#ifndef EXTRA_H
#define EXTRA_H

#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <Eigen/Geometry>

#ifndef M_PI
#define M_PI  3.14159265358979
#endif

using namespace Eigen;


inline void glMultMatrix( const Matrix4f& m )
{
    glMultMatrixf( m.data() );
}

#endif
