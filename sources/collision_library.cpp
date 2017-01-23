#include "collision_library.h"
#include <iostream>
#include <cmath>

namespace collision
{


    CollisionState
    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S0,
                     const DynamicPhysObject<GMlib::PSphere<float>>& S1,
                     seconds_type                                    dt)
    {

        auto max_dt = dt;
        auto min_dt = max(S0.curr_t_in_dt, S1.curr_t_in_dt );
        auto new_dt = max_dt -min_dt;
        auto p0 = S0.getMatrixToScene() *S0.getPos().toType<double>();
        auto r0 = S0.getRadius();
        auto p1 = S1.getMatrixToScene() *S1.getPos().toType<double>();
        auto r1 = S1.getRadius();
        auto r = r0 + r1;
        auto Q = p1 - p0;
        auto R = S1.computeTrajectory(new_dt) - S0.computeTrajectory(new_dt);
        auto a = R * R;
        auto b = Q * R;
        auto c = (Q * Q) - r*r;
        auto epsilon = 1e-6;

        if ((std::abs(c))< epsilon)
        {
            return(CollisionState(seconds_type (0.0),CollisionStateFlag::SingularityParallelAndTouching));
        }
        else if ((std::abs(a))< epsilon)
        {
            return(CollisionState(seconds_type (0.0),CollisionStateFlag::SingularityParallel));
        }
        else if ((b*b - a*c)< 0)
        {
            return(CollisionState(seconds_type (0.0),CollisionStateFlag::SingularityNoCollision));
        }
        else
        {
            auto x = (-b - sqrt(b*b - a*c))/a;
            return (CollisionState (x*new_dt +min_dt,CollisionStateFlag::Collision));
        }

    }

    CollisionState
    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S,
                     const StaticPhysObject<GMlib::PPlane<float>>&   P,
                     seconds_type                                    dt)
    {

        auto max_dt = dt;
        auto min_dt = S.curr_t_in_dt;
        auto new_dt = max_dt -min_dt;
        auto p = S.getMatrixToScene() * S.getPos();
        auto r = S.getRadius();

        auto unconst_P = const_cast <StaticPhysObject<GMlib::PPlane<float>>&>(P);
        const auto M = unconst_P.evaluateParent(0.5f,0.5f,1,1);
        auto q = M(0)(0);
        auto u = M(1)(0);
        auto v = M(0)(1);
        auto n = GMlib::Vector<float,3>(u ^ v).getNormalized();
        auto d = (q + r * n) - p;

        auto ds = S.computeTrajectory(new_dt);
        auto epsilon = 1e-6;

        if ((std::abs(d * n))< epsilon)
                {
                    return(CollisionState(seconds_type (0.0),CollisionStateFlag::SingularityParallelAndTouching));
                }

        else if ((std::abs(ds * n))< epsilon)
        {
            return(CollisionState(seconds_type (0.0),CollisionStateFlag::SingularityParallel));
        }

        else
        {
            auto x = (d * n) / (ds * n);
            return (CollisionState (x*new_dt +min_dt,CollisionStateFlag::Collision));
        }

    }


    CollisionState
    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S,
                     const StaticPhysObject<GMlib::PBezierSurf<float>>& B,
                     seconds_type                                    dt)
    {

        auto max_dt = dt;
        auto min_dt = S.curr_t_in_dt;
        auto new_dt = max_dt -min_dt;
        float u,v,t,delta_u,delta_v,delta_t;
        u = 0.5;
        v = 0.5;
        t = 0.0;
        delta_u = 0.5;
        delta_v = 0.5;
        delta_t = 0.0;
        auto p0 = S.getMatrixToScene() *S.getPos();
        auto r = S.getRadius();
        auto unconst_B = const_cast <StaticPhysObject<GMlib::PBezierSurf<float>>&>(B);
        GMlib::Vector<double, 3> ds = S.computeTrajectory(new_dt);
        GMlib::SqMatrix<double,3> A;

        //iteration

        for (int i=0; i<=5;i++){
        auto p = p0 + ds*t;
        const auto M = unconst_B.evaluateParent(u,v,1,1);
        auto q = M(0)(0);
        const auto Su = M(1)(0);
        const  auto Sv = M(0)(1);
        auto Sn = GMlib::Vector<float,3> (Su ^ Sv).getNormalized();
        A.setCol(Su,0);
        A.setCol(Sv,1);
        A.setCol(-ds,2);
        GMlib::SqMatrix<float,3> A_inv = A;
        A_inv.invert();
        GMlib::APoint<float, 3> b = GMlib::Vector<float, 3> {p-q-Sn*r};
        GMlib::APoint<float, 3> X = A_inv*b;
        delta_u = X(0);
        delta_v = X(1);
        delta_t = X(2);
        u += delta_u;
        v += delta_v;
        t += delta_t;

        auto epsilon = 1e-5;

        if ( (delta_t < epsilon) && (std::abs(delta_u) < epsilon) && (std::abs(delta_v)< epsilon))
        {
            return (CollisionState (seconds_type(delta_t),CollisionStateFlag::Collision));
        }

        }
        return(CollisionState(seconds_type (0.0),CollisionStateFlag::SingularityNoCollision));

    }


    CollisionState
    detectCollision (const DynamicPSphere&  S,
                     const StaticPCylinder& C,
                     seconds_type dt)
{

    auto max_dt = dt;
    auto min_dt = S.curr_t_in_dt;
    auto new_dt = max_dt -min_dt;
    auto p = S.getMatrixToScene() * S.getPos();
    auto r_s = S.getRadius();
    auto r_c = S.getRadius();
    auto unconst_C = const_cast <StaticPhysObject<GMlib::PCylinder<float>>&>(C);
    const auto M = unconst_C.evaluateParent(0.5f,0.5f,1,1);
    auto q = M(0)(0);
    auto u = M(1)(0);
    auto v = M(0)(1);
    auto n = GMlib::Vector<float,3>(u ^ v).getNormalized();
    auto d = (q + (r_s + r_c) * n) - p;

    auto ds = S.computeTrajectory(new_dt);
    auto epsilon = 1e-6;

    if ((std::abs(d * n))< epsilon)
            {
                return(CollisionState(seconds_type (0.0),CollisionStateFlag::SingularityParallelAndTouching));
            }

    else if ((std::abs(ds * n))< epsilon)
    {
        return(CollisionState(seconds_type (0.0),CollisionStateFlag::SingularityParallel));
    }

    else
    {
        auto x = (d * n) / (ds * n);
        return (CollisionState (x*new_dt +min_dt,CollisionStateFlag::Collision));
    }

}



    void
    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S0,
                           DynamicPhysObject<GMlib::PSphere<float>>& S1,
                           seconds_type                              dt)
    {
        /*auto p0 = S0.getPos();
        auto r0 = S0.getRadius();
        auto p1 = S1.getPos();
        auto r1 = S1.getRadius();

        auto d = GMlib::Vector<float,3>(p1 -p0);
        auto n = GMlib::Vector<float,3>(d ^ d.getLinIndVec()).getNormalized();
        auto v0d = (S0.velocity * d);
        auto v1d = (S1.velocity * d);
        auto v0n = v0d * n;
        auto v1n = v1d * n;
        auto new_v0d = v0d * (S0.mass - S1.mass)/(S0.mass + S1.mass)
                     + v1d * 2 * S1.mass / (S0.mass + S1.mass);
        auto new_v1d = v1d * (S1.mass - S0.mass)/(S0.mass + S1.mass)
                     + v0d * 2 * S0.mass / (S0.mass + S1.mass);

        S0.velocity = GMlib::Vector<float,3>(v0n * n) + (new_v0d * d.getNormalized());
        S1.velocity = GMlib::Vector<float,3>(v1n * n) + (new_v1d * d.getNormalized());*/

        const auto S0_old_vel = S0.velocity;
        const auto S1_old_vel = S1.velocity;
        const auto S0_pos = S0.getPos().toType<double>();
        const auto S1_pos = S1.getPos().toType<double>();
        const auto S0_mass = S0.mass;
        const auto S1_mass = S1.mass;
        const auto distance_vector_d = GMlib::Vector<double,3>(S1_pos - S0_pos);
        const auto normal_d = distance_vector_d.getNormalized();
        const auto n = (GMlib::Vector<double,3>(distance_vector_d).getLinIndVec()).getNormalized();
        const auto v0_d = (S0_old_vel * normal_d);
        const auto v1_d = (S1_old_vel * normal_d);
        const auto v0_n = (S0_old_vel * n);
        const auto v1_n = (S1_old_vel * n);
        const auto new_v0_d = (((S0_mass - S1_mass) / (S0_mass + S1_mass) ) * v0_d ) + (((2 * S1_mass) / (S0_mass + S1_mass) ) * v1_d );
        const auto new_v1_d = (((S1_mass - S0_mass) / (S0_mass + S1_mass) ) * v1_d ) + (((2 * S0_mass) / (S0_mass + S1_mass) ) * v0_d );
        const auto S0_new_vel = (v0_n * n) + (new_v0_d * normal_d);
        auto S1_new_vel = v1_n * n + new_v1_d * normal_d;
        S0.velocity = S0_new_vel;
        S1.velocity = S1_new_vel;

    }

    void
    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S,
                           const StaticPhysObject<GMlib::PPlane<float>>&   P,
                           seconds_type                              dt)
    {

        auto unconst_P = const_cast <StaticPhysObject<GMlib::PPlane<float>>&>(P);
        const auto M = unconst_P.evaluateParent(0.5f,0.5f,1,1);
        auto u = M(1)(0);
        auto v = M(0)(1);
        auto n = GMlib::Vector<float,3>(u ^ v).getNormalized();
        auto vel = S.velocity * n;

        auto new_velocity = S.velocity - ((2* vel)*n);
        S.velocity = new_velocity ;
    }


    std::unique_ptr<Controller> unittestCollisionControllerFactory(){ return std::make_unique<MyController> (); }
    //void DynamicPhysObject<GMlib::PSphere<float> >::simulateToTInDt(seconds_type t){ }

    GMlib::Vector<double,3> DynamicPhysObject<GMlib::PSphere<float> >::externalForces() const {
        assert (environment != nullptr);
        return this->environment->externalForces().toType<double>();

    }

  /* template <class Container_T >
    void sortAndMakeUnique( Container_T & container) {

        std::sort(container.begin(),container.end(), [] (const auto & a , const auto & b) {
            return a.t_in_dt <b.t_in_dt;
        });

        //make unique
    }*/


    void MyController::localSimulate(double dt) { }
} // END namespace collision

