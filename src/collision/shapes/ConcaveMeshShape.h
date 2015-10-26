/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H
#define REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H

// Libraries
#include "ConcaveShape.h"
#include "collision/TriangleMesh.h"

namespace reactphysics3d {

// TODO : Implement raycasting with this collision shape

// TODO : Make possible for the user to have a scaling factor on the mesh

// Class ConcaveMeshShape
/**
 * This class represents a concave mesh shape. Note that collision detection
 * with a concave mesh shape can be very expensive. You should use only use
 * this shape for a static mesh.
 */
class ConcaveMeshShape : public ConcaveShape {

    protected:

        // -------------------- Attributes -------------------- //

        /// Triangle mesh
        TriangleMesh* mTriangleMesh;

        /// Mesh minimum bounds in the three local x, y and z directions
        Vector3 mMinBounds;

        /// Mesh maximum bounds in the three local x, y and z directions
        Vector3 mMaxBounds;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConcaveMeshShape(const ConcaveMeshShape& shape);

        /// Private assignment operator
        ConcaveMeshShape& operator=(const ConcaveMeshShape& shape);

        /// Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction,
                                                       void** cachedCollisionData) const;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                          void** cachedCollisionData) const;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

        /// Recompute the bounds of the mesh
        // TODO : Check if we need this when AABB tree is used
        void recalculateBounds();

    public:

        /// Constructor
        ConcaveMeshShape(TriangleMesh* triangleMesh);

        /// Destructor
        ~ConcaveMeshShape();

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const;
};

// Return the number of bytes used by the collision shape
inline size_t ConcaveMeshShape::getSizeInBytes() const {
    return sizeof(ConcaveMeshShape);
}

// Return a local support point in a given direction with the object margin
inline Vector3 ConcaveMeshShape::getLocalSupportPointWithMargin(const Vector3& direction,
                                                           void** cachedCollisionData) const {

    // TODO : Implement this
    return Vector3(0, 0, 0);
}

// Return a local support point in a given direction without the object margin
inline Vector3 ConcaveMeshShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                              void** cachedCollisionData) const {
    // TODO : Implement this
    return Vector3(0.0, 0.0, 0.0);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void ConcaveMeshShape::getLocalBounds(Vector3& min, Vector3& max) const {

    min = mMinBounds;
    max = mMaxBounds;
}

// Return the local inertia tensor of the sphere
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void ConcaveMeshShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {

    // Default inertia tensor
    // Note that this is not very realistic for a concave triangle mesh.
    // However, in most cases, it will only be used static bodies and therefore,
    // the inertia tensor is not used.
    tensor.setAllValues(mass, 0, 0,
                        0, mass, 0,
                        0, 0, mass);
}

// Return true if a point is inside the collision shape
inline bool ConcaveMeshShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {

    // TODO : Implement this
    return false;
}

}
#endif

