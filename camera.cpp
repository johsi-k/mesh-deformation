#include "camera.h"
#include "extra.h"
#include <GL/glu.h>
#include <iostream>

using namespace std;
using namespace Eigen;

Camera::Camera()
{
	mStartRot = Matrix4f::Identity();
    mCurrentRot = Matrix4f::Identity();
}

void Camera::SetDimensions(int w, int h)
{
    mDimensions[0] = w;
    mDimensions[1] = h;
}

void Camera::SetPerspective(float fovy)
{
    mPerspective[0] = fovy;
}

void Camera::SetViewport(int x, int y, int w, int h)
{
    mViewport[0] = x;
    mViewport[1] = y;
    mViewport[2] = w;
    mViewport[3] = h;
    mPerspective[1] = float( w ) / h;
}

void Camera::SetCenter(const Vector3f& center)
{
    mStartCenter = mCurrentCenter = center;
}

void Camera::SetRotation(const Matrix4f& rotation)
{
    mStartRot = mCurrentRot = rotation;
}

void Camera::SetDistance(const float distance)
{
    mStartDistance = mCurrentDistance = distance;
}

void Camera::MouseClick(Button button, int x, int y)
{
    mStartClick[0] = x;
    mStartClick[1] = y;

    mButtonState = button;
    switch (button)
    {
    case LEFT:
        mCurrentRot = mStartRot;
        break;
    case MIDDLE:
        mCurrentCenter = mStartCenter;
        break;
    case RIGHT:
        mCurrentDistance = mStartDistance;
        break;        
    default:
        break;
    }
}

void Camera::MouseDrag(int x, int y)
{
    switch (mButtonState)
    {
    case LEFT:
        ArcBallRotation(x,y);
        break;
    case MIDDLE:
        PlaneTranslation(x,y);
        break;
    case RIGHT:
        DistanceZoom(x,y);
        break;
    default:
        break;
    }
}


void Camera::MouseRelease(int x, int y)
{
    mStartRot = mCurrentRot;
    mStartCenter = mCurrentCenter;
    mStartDistance = mCurrentDistance;
    
    mButtonState = NONE;
}

Matrix4f generateRotationMatrix(const Vector3f& rDirection, float radians)
{
	Vector3f normalizedDirection = rDirection.normalized();

	float cosTheta = cos(radians);
	float sinTheta = sin(radians);

	float x = normalizedDirection.x();
	float y = normalizedDirection.y();
	float z = normalizedDirection.z();

	Matrix <float, 4, 4, ColMajor> m;
	m <<
		x * x * (1.0f - cosTheta) + cosTheta, y * x * (1.0f - cosTheta) - z * sinTheta, z * x * (1.0f - cosTheta) + y * sinTheta, 0.0f,
		x * y * (1.0f - cosTheta) + z * sinTheta, y * y * (1.0f - cosTheta) + cosTheta, z * y * (1.0f - cosTheta) - x * sinTheta, 0.0f,
		x * z * (1.0f - cosTheta) - y * sinTheta, y * z * (1.0f - cosTheta) + x * sinTheta, z * z * (1.0f - cosTheta) + cosTheta, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;
	return m;
}



void Camera::ArcBallRotation(int x, int y)
{
    float sx, sy, sz, ex, ey, ez;
    float scale;
    float sl, el;
    float dotprod;
    
    // find vectors from center of window
    sx = mStartClick[0] - ( mDimensions[0] / 2.f );
    sy = mStartClick[1] - ( mDimensions[1] / 2.f );
    ex = x - ( mDimensions[0] / 2.f );
    ey = y - ( mDimensions[1] / 2.f );
    
    // invert y coordinates (raster versus device coordinates)
    sy = -sy;
    ey = -ey;
    
    // scale by inverse of size of window and magical sqrt2 factor
    if (mDimensions[0] > mDimensions[1]) {
        scale = (float) mDimensions[1];
    } else {
        scale = (float) mDimensions[0];
    }

    scale = 1.f / scale;
    
    sx *= scale;
    sy *= scale;
    ex *= scale;
    ey *= scale;

    // project points to unit circle
    sl = hypot(sx, sy);
    el = hypot(ex, ey);
    
    if (sl > 1.f) {
        sx /= sl;
        sy /= sl;
        sl = 1.0;
    }
    if (el > 1.f) {
        ex /= el;
        ey /= el;
        el = 1.f;
    }
    
    // project up to unit sphere - find Z coordinate
    sz = sqrt(1.0f - sl * sl);
    ez = sqrt(1.0f - el * el);
    
    // rotate (sx,sy,sz) into (ex,ey,ez)
    
    // compute angle from dot-product of unit vectors (and double it).
    // compute axis from cross product.
    dotprod = sx * ex + sy * ey + sz * ez;

    if( dotprod != 1 )
    {
        Vector3f axis = Vector3f( sy * ez - ey * sz, sz * ex - ez * sx, sx * ey - ex * sy );
        axis.normalize();
        
        float angle = 2.0f * acos( dotprod );

        mCurrentRot = generateRotationMatrix( axis, angle );
        mCurrentRot = mCurrentRot * mStartRot;
    }
    else
    {
        mCurrentRot = mStartRot;
    }


}

void Camera::PlaneTranslation(int x, int y)
{
    // map window x,y into viewport x,y

    // start
    int sx = mStartClick[0] - mViewport[0];
    int sy = mStartClick[1] - mViewport[1];

    // current
    int cx = x - mViewport[0];
    int cy = y - mViewport[1];


    // compute "distance" of image plane (wrt projection matrix)
    float d = float(mViewport[3])/2.0f / tan(mPerspective[0]*M_PI / 180.0f / 2.0f);

    // compute up plane intersect of clickpoint (wrt fovy)
    float su = -sy + mViewport[3]/2.0f;
    float cu = -cy + mViewport[3]/2.0f;

    // compute right plane intersect of clickpoint (ASSUMED FOVY is 1)
    float sr = (sx - mViewport[2]/2.0f);
    float cr = (cx - mViewport[2]/2.0f);

    Vector2f move(cr-sr, cu-su);

    // this maps move
    move *= -mCurrentDistance/d;

    mCurrentCenter = mStartCenter +
        + move[0] * Vector3f(mCurrentRot(0,0),mCurrentRot(0,1),mCurrentRot(0,2))
        + move[1] * Vector3f(mCurrentRot(1,0),mCurrentRot(1,1),mCurrentRot(1,2));

}

Ray Camera::generateRay(float x, float y) {
	float x_space = (2 * x / mDimensions[0]) - 1;
	float y_space = 1 - (2 * y / mDimensions[1]);
	float proj_matrix[16];


	Matrix<float,4,4,ColMajor> lookAtDisplacement; 
	lookAtDisplacement <<
		1, 0, 0, 0, 
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, -mCurrentDistance, 1;

	Matrix <float,4,4,ColMajor> translationMatrix;

	translationMatrix <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		mCurrentCenter.x(), mCurrentCenter.y(), mCurrentCenter.z(), 1;

	glGetFloatv(GL_PROJECTION_MATRIX, proj_matrix);

	Matrix4f cameraTransform = lookAtDisplacement * mCurrentRot * translationMatrix;
	Matrix4f projectionMatrix = Matrix4f::Identity();

	for (int i = 0; i < 16; i++) {
		projectionMatrix(i) = proj_matrix[i];
	}

	Vector4f mouseClip = Vector4f(x_space, y_space, 0, 1);

	Vector4f mouse = cameraTransform.inverse() * projectionMatrix.inverse() * mouseClip;
	Vector3f directionFromMouse = Vector3f(mouse.x(), mouse.y(), mouse.z()).normalized();

	Vector4f pTrans = mCurrentRot.inverse() * Vector4f(0, 0, mCurrentDistance, 1) + Vector4f(mCurrentCenter.x(), mCurrentCenter.y(), mCurrentCenter.z(), 1);
	Ray r = Ray(Vector3f(pTrans.x(),pTrans.y(),pTrans.z()), directionFromMouse);

	return r;
}

void Camera::ApplyViewport() const
{
    glViewport(mViewport[0],mViewport[1],mViewport[2],mViewport[3]);
}

void Camera::ApplyPerspective() const
{
    gluPerspective(mPerspective[0], mPerspective[1], 1.0, 1000.0);   
}


void Camera::ApplyModelview() const
{

    // back up distance
    gluLookAt(0,0,mCurrentDistance,
              0,0,0,
              0.0, 1.0, 0.0);

    // rotate object
    glMultMatrix(mCurrentRot);

    //translate object to center
    glTranslatef(-mCurrentCenter.x(),-mCurrentCenter.y(),-mCurrentCenter.z());    
}

void Camera::DistanceZoom(int x, int y)
{
    int sy = mStartClick[1] - mViewport[1];
    int cy = y - mViewport[1];

    float delta = float(cy-sy)/mViewport[3];

    // exponential zoom factor
    mCurrentDistance = mStartDistance * exp(delta);  
}

