#include "collision_library.h"



namespace collision
{


    CollisionState
    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S0,
                     const DynamicPhysObject<GMlib::PSphere<float>>& S1,
                     seconds_type                                    dt)
    {
    }

    CollisionState
    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S0,
                     const StaticPhysObject<GMlib::PSphere<float>>&  S1,
                     seconds_type                                    dt)
    {
    }

    CollisionState
    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S,
                     const StaticPhysObject<GMlib::PPlane<float>>&   P,
                     seconds_type                                    dt)
    {
    }

    CollisionState
    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>&  S,
                     const StaticPhysObject<GMlib::PCylinder<float>>& C,
                     seconds_type                                    dt)
    {
    }



    void
    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S0,
                           DynamicPhysObject<GMlib::PSphere<float>>& S1,
                           seconds_type                              dt)
    {
    }

    void
    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S0,
                           StaticPhysObject<GMlib::PSphere<float>>&  S1,
                           seconds_type                              dt)
    {
    }

    void
    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S,
                           StaticPhysObject<GMlib::PPlane<float>>&   P,
                           seconds_type                              dt)
    {
    }

    void
    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>&  S,
                           StaticPhysObject<GMlib::PCylinder<float>>& C,
                           seconds_type                              dt)
    {
    }
} // END namespace collision

