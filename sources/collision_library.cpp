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

        auto new_velocity = S.velocity - ((2* vel)*n)*0.95;
        S.velocity = new_velocity ;
    }


    std::unique_ptr<Controller> unittestCollisionControllerFactory(){ return std::make_unique<collision_controller> (); }
    //void DynamicPhysObject<GMlib::PSphere<float> >::simulateToTInDt(seconds_type t){ }

    GMlib::Vector<double,3> DynamicPhysObject<GMlib::PSphere<float> >::externalForces() const {
        assert (environment != nullptr);
        return this->environment->externalForces().toType<double>();

    }


    void collision_controller::localSimulate(double dt) {

        //we need to reset currentTInDt of all dynamic objects

        for(auto& sphere : _dynamic_spheres){
            sphere->curr_t_in_dt =seconds_type(0.0);
        }




        //Detect collisions
        detectCollisions(dt);
        sortAndMakeUnique(_collisions);

        while (!_collisions.empty()){

            auto col = _collisions.begin();
            auto col_time =  col->first;
            auto col_obj1 = col->second.obj1;
            auto col_obj2 = col->second.obj2;
            _collisions.erase(col); //col is now invalid but it won't be used until next iteration when we redefine it
            if (auto sphere2 = dynamic_cast<DynamicPSphere*> (col_obj2)){
                auto sphere1 = dynamic_cast<DynamicPSphere*> (col_obj1);
                sphere1->simulateToTInDt(col_time);
                sphere2->simulateToTInDt(col_time);
                computeImpactResponse(*sphere1,*sphere2,col_time);
            }

            else {
                auto sphere = dynamic_cast<DynamicPSphere*> (col_obj1);
                auto plane = dynamic_cast<StaticPPlane*> (col_obj2);
                sphere->simulateToTInDt(col_time);
                computeImpactResponse(*sphere,*plane,col_time);
            }

            //Detect more collisions
            detectCollisions(dt);
            sortAndMakeUnique(_collisions);

        }
        for(auto& sphere : _dynamic_spheres){
            sphere->simulateToTInDt(seconds_type(dt));
        }

    }
    void DynamicPhysObject<GMlib::PSphere<float> >::simulateToTInDt( seconds_type t ) {

        auto t0 = seconds_type(t - this->curr_t_in_dt);
        auto Mi = this->getMatrixToSceneInverse();
        //move

        auto ds = this->computeTrajectory(t0);
        this->translateParent(Mi*ds);
        this->curr_t_in_dt =t;
        //update physics
        auto F = this->externalForces();
        auto c = t0.count();
        auto a = F*c;
        this->velocity += a;


    }
    void collision_controller::add (DynamicPSphere* const sphere) {
        _dynamic_spheres.push_back(sphere);
        sphere->environment = &_environment;
    }

    void collision_controller::detectCollisions(double dt){

        //loop for collision between dynamic objects (only spheres for now)
        for (auto it1 = _dynamic_spheres.begin() ; it1 != _dynamic_spheres.end() ; ++it1){
            for (auto it2 = it1+1 ; it2 != _dynamic_spheres.end() ; ++it2 ){
                auto col = collision::detectCollision(**it1,**it2,seconds_type(dt));
                const auto &sphere1= *it1;
                const auto &sphere2= *it2;
                auto min_ctidt = std::max(sphere1->curr_t_in_dt, sphere2->curr_t_in_dt);
                if (col.flag == CollisionStateFlag::Collision && col.time < seconds_type(dt) && col.time > min_ctidt ){
                    _collisions.emplace(col.time,CollisionObject(sphere1,sphere2,col.time));
                }
            }
        }

        //loop for collision with static objects (only dynamic spheres with static planes for now)
        for (auto &it1 : _dynamic_spheres){
            for (auto &it2 : _static_planes){
                auto col = collision::detectCollision(*it1,*it2,seconds_type(dt));
                if (col.flag == CollisionStateFlag::Collision && col.time < seconds_type(dt) && col.time > it1->curr_t_in_dt){
                    _collisions.emplace(col.time,CollisionObject(it1,it2,col.time));
                }
               else if(col.flag ==CollisionStateFlag::SingularityParallelAndTouching){
                    setAttachedObjects(it1,it2);
                }
            }
        }
    }

    stateChangeObject detectStateChanges(DynamicPhysObject<GMlib::PSphere<float>> *  sphere, double dt){
//        auto P =  sphere->getAttachedPlane();
//        auto M = P->evaluateParent(0.5f,0.5f,1,1);
//        auto u = M(1)(0);
//        auto v = M(0)(1);
//        auto q = M(0)(0);
//        auto n = GMlib::Vector<float,3>(u ^ v).getNormalized();
//        auto epsilon = 1e-5;
//        auto dts = seconds_type(dt);

//        auto max_dt = dts;
//        auto min_dt = sphere->curr_t_in_dt;
//        auto new_dt = max_dt -min_dt;
//        auto ds = sphere->computeTrajectory(new_dt);

//        if (sphere->state == states::Rolling){
//            if (ds * n > 0)
//                return (stateChangeObject(sphere, states::Free));
//            else if (ds * n < 0 && std::abs(ds * n) < epsilon )
//                return (stateChangeObject(sphere,states::Still)) ;
//            else
//                return (stateChangeObject(sphere,  states::NoChange));
//        }

//        else if (sphere->state == states::Still){
//            if (ds * n <= 0 && std::abs(ds * n) > epsilon){
//                //Correct trajectory
//                return (stateChangeObject(sphere,  states::Rolling));
//            }
//            else if (ds * n > 0 && std::abs(ds * n) > epsilon )
//                return (stateChangeObject(sphere,  states::Free));
//            else
//                return (stateChangeObject(sphere,  states::NoChange));
//        }

//        else{
//            auto r = sphere->getRadius();
//            auto p = sphere->getMatrixToScene() * sphere->getPos();
//            auto d = (q + r * n) - p;
//            if (ds * n <= 0 && std::abs(((-n*r) * ds) -(ds*ds)) < epsilon){
//                //Correct trajectory
//                return (stateChangeObject(sphere,  states::Rolling));
//            }
//            else if (std::abs(d * n) < epsilon )
//                return (stateChangeObject(sphere,  states::Still));
//            else
//                return (stateChangeObject(sphere,  states::NoChange));

//        }
}


    std::vector<StaticPPlane*>  collision_controller:: getAttachedObjects (DynamicPSphere* sphere) {

         return (_attachedObjects[sphere]);

    }

    void collision_controller::setAttachedObjects(DynamicPSphere *sphere, StaticPPlane *plane){

        _attachedObjects[sphere].push_back(plane);
    }


   // void correctTrajectory(const DynamicPhysObject<GMlib::PSphere<float>>& S, seconds_type dt);


} // END namespace collision

