#include "collision_library.h"
#include <iostream>
#include <cmath>

namespace collision
{


    CollisionState
    detectCollision ( DynamicPhysObject<GMlib::PSphere<float>>& S0,
                      DynamicPhysObject<GMlib::PSphere<float>>& S1,
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
        auto ds0 =S0.computeTrajectory(new_dt);
        auto ds1 =S1.computeTrajectory(new_dt);
        if(S0.state == states::Rolling){
           ds0 =S0.adjustTrajectory(new_dt);
        }

        if(S1.state == states::Rolling){
           ds1 =S1.adjustTrajectory(new_dt);
        }
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
    detectCollision ( DynamicPhysObject<GMlib::PSphere<float>>& S,
                      StaticPhysObject<GMlib::PPlane<float>>&   P,
                     seconds_type                                    dt)
    {

        auto max_dt = dt;
        auto min_dt = S.curr_t_in_dt;
        auto new_dt = max_dt -min_dt;
        auto p = S.getMatrixToScene() * S.getPos();
        auto r = S.getRadius();

       // auto unconst_P = const_cast <StaticPhysObject<GMlib::PPlane<float>>&>(P);
        const auto M = P.evaluateParent(0.5f,0.5f,1,1);
        auto q = M(0)(0);
        auto u = M(1)(0);
        auto v = M(0)(1);
        auto n = GMlib::Vector<float,3>(u ^ v).getNormalized();
        auto d = (q + r * n) - p;
        auto ds = S.computeTrajectory(new_dt);
        if(S.state == states::Rolling){
            //auto unconst_S = const_cast < DynamicPhysObject<GMlib::PSphere<float>>&>(S);
            ds = S.adjustTrajectory(new_dt);
        }

        auto epsilon = 1e-5;

        if ((std::abs(d * n))< epsilon)
                {
                    return(CollisionState(seconds_type(epsilon),CollisionStateFlag::SingularityParallelAndTouching));
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

        auto epsilon=1e-5;
        auto unconst_P = const_cast <StaticPhysObject<GMlib::PPlane<float>>&>(P);
        const auto M = unconst_P.evaluateParent(0.5f,0.5f,1,1);
        auto u = M(1)(0);
        auto v = M(0)(1);
        auto n = GMlib::Vector<float,3>(u ^ v).getNormalized();
        auto vel = S.velocity * n;

        auto new_velocity = S.velocity - ((2* vel)*n)*0.95;
        S.velocity = new_velocity ;
        std::cout<<new_velocity << "state is : " << int(S.state)<<std::endl;
    }


    std::unique_ptr<Controller> unittestCollisionControllerFactory(){ return std::make_unique<collision_controller> (); }

    GMlib::Vector<double,3> DynamicPhysObject<GMlib::PSphere<float> >::externalForces() const {
        assert (environment != nullptr);
        return this->environment->externalForces().toType<double>();

    }


    void collision_controller::localSimulate(double dt) {

        //we need to reset currentTInDt of all dynamic objects

        for(auto& sphere : _dynamic_spheres){
            sphere->curr_t_in_dt =seconds_type(0.0);
        }

        //Detect state changes

        detectStateChange(dt);

        //Detect collisions
        detectCollisions(dt);
        sortAndMakeUnique(_collisions);
        sortAndMakeUniqueState(_stateChanges);
         if (!_collisions.empty() && !_stateChanges.empty() )
             sortAndMakeUnique(_collisions,_stateChanges);


        while (!_collisions.empty() || !_stateChanges.empty()){

            if (!_collisions.empty() && !_stateChanges.empty() ){
                auto col = _collisions.begin();
                auto col_time =  col->first;
                auto singularity = _stateChanges.begin();
                auto sing_time =  singularity->first;

                if( col_time<sing_time){
                        //Resolve collision
                        auto col_obj1 = col->second.obj1;
                        auto col_obj2 = col->second.obj2;
                        _collisions.erase(col); //col is now invalid but it won't be used until next iteration when we redefine it
                        if (auto sphere2 = dynamic_cast<DynamicPSphere*> (col_obj2)){
                            auto sphere1 = dynamic_cast<DynamicPSphere*> (col_obj1);
                            sphere1->simulateToTInDt(col_time);
                            if(sphere2->state != states::Still)
                                sphere2->simulateToTInDt(col_time);
                            else{
                                sphere2->curr_t_in_dt = col_time;
//                                sphere2->velocity = {0.0f,0.0f,0.0f};
//                                sphere2->environment = &_noGravity;
                            }
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
                           //Detect more state changes
                           detectStateChange(dt);
                           sortAndMakeUnique(_collisions);
                           sortAndMakeUniqueState(_stateChanges);
                           if (!_collisions.empty() && !_stateChanges.empty() )
                               sortAndMakeUnique(_collisions,_stateChanges);
                }

                       //Resolve state changes
                else {
                    if (_stateChanges.empty())
                        std::cout<<"empty container";
                    std::cout<<"state changes from" << int(singularity->second.obj->state)<<"   to  "<<int(singularity->second.stateChanges)<<std::endl ;

                    if (singularity->second.stateChanges != states::Still)
                        singularity->second.obj->simulateToTInDt(sing_time);
                    else{
                        singularity->second.obj->curr_t_in_dt = sing_time;
                        singularity->second.obj->velocity = {0.0f,0.0f,0.0f};
                        singularity->second.obj->environment = &_noGravity;
                    }
                        if (singularity->second.stateChanges == states::Free){
                            _attachedPlanes[ singularity->second.obj].clear();
                        }
                        else
                            _attachedPlanes[ singularity->second.obj] = singularity->second.attachedPlanes;

                      singularity->second.obj->state = singularity->second.stateChanges;
                    _stateChanges.erase(singularity);
                    //Detect more collisions
                    detectCollisions(dt);
                    //Detect more state changes
                    detectStateChange(dt);
                    sortAndMakeUnique(_collisions);
                    sortAndMakeUniqueState(_stateChanges);
                    if (!_collisions.empty() && !_stateChanges.empty() )
                        sortAndMakeUnique(_collisions,_stateChanges);
                }
            }
            else if (!_collisions.empty()){
                //Resolve collision
                auto col = _collisions.begin();
                auto col_time =  col->first;
                auto col_obj1 = col->second.obj1;
                auto col_obj2 = col->second.obj2;
                _collisions.erase(col); //col is now invalid but it won't be used until next iteration when we redefine it
                if (auto sphere2 = dynamic_cast<DynamicPSphere*> (col_obj2)){
                    auto sphere1 = dynamic_cast<DynamicPSphere*> (col_obj1);
                    sphere1->simulateToTInDt(col_time);
                    if(sphere2->state != states::Still)
                        sphere2->simulateToTInDt(col_time);
                    else{
                        sphere2->curr_t_in_dt = col_time;
//                        sphere2->velocity = {0.0f,0.0f,0.0f};
//                        sphere2->environment = &_noGravity;
                    }
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
                   //Detect more state changes
                   detectStateChange(dt);
                   sortAndMakeUnique(_collisions);
                   sortAndMakeUniqueState(_stateChanges);
                   if (!_collisions.empty() && !_stateChanges.empty() )
                       sortAndMakeUnique(_collisions,_stateChanges);
            }
            else {
                auto singularity = _stateChanges.begin();
                auto sing_time =  singularity->first;
                std::cout<<"state changes from" << int(singularity->second.obj->state)<<"   to  "<<int(singularity->second.stateChanges)<<std::endl ;
                if (singularity->second.stateChanges != states::Still)
                    singularity->second.obj->simulateToTInDt(sing_time);
                else{
                    singularity->second.obj->curr_t_in_dt = sing_time;
                    singularity->second.obj->velocity = {0.0f,0.0f,0.0f};
                    singularity->second.obj->environment = &_noGravity;
                }
                    if (singularity->second.stateChanges == states::Free){
                        _attachedPlanes[ singularity->second.obj].clear();
                    }
                    else
                        _attachedPlanes[ singularity->second.obj] = singularity->second.attachedPlanes;

                  singularity->second.obj->state = singularity->second.stateChanges;
                _stateChanges.erase(singularity);
                //Detect more collisions
                detectCollisions(dt);
                //Detect more state changes
                detectStateChange(dt);
                sortAndMakeUnique(_collisions);
                sortAndMakeUniqueState(_stateChanges);
                if (!_collisions.empty() && !_stateChanges.empty() )
                    sortAndMakeUnique(_collisions,_stateChanges);

            }


        }

        for(auto& sphere : _dynamic_spheres){
            if (sphere->state != states::Still)
                sphere->simulateToTInDt(seconds_type(dt));
            else
               sphere->curr_t_in_dt = seconds_type(dt);

        }

    }
    void DynamicPhysObject<GMlib::PSphere<float> >::simulateToTInDt( seconds_type t ) {

//        if  ((this->state == states::Still || this->state == states::Rolling)  && std::abs(this->velocity(2))  <= 1e-2){
//            this->curr_t_in_dt =t;
//            this->velocity = {0.0f,0.0f,0.0f};
//            this->state  == states::Still;
//            this->environment = &this->sphereController->_noGravity;
//        }
//        else if  (this->state == states::Free && this->velocity  <= 1e-5){
//            this->velocity = {0.0f,0.0f,0.0f};
//            this->state  == states::Still;
//            this->environment = &this->sphereController->_noGravity;
//        }
//       else {
            auto t0 = seconds_type(t - this->curr_t_in_dt);
            auto Mi = this->getMatrixToSceneInverse();
            //move
            auto planes = this->sphereController->getAttachedPlanes(this);
            GMlib::Vector <float,3>n {0.0f,0.0f,0.0f};
            for (auto &it :planes){
                auto M = it->evaluateParent(0.5f,0.5f,1,1);
                auto u = M(1)(0);
                auto v = M(0)(1);
                auto normal = GMlib::Vector<float,3>(u ^ v);
                n+=normal;
           }
           n= GMlib::Vector <float,3>(n/planes.size()).getNormalized();

           GMlib::Vector<double,3>  ds = (0.0f,0.0f,0.0f);
           if ((this->state == states::Rolling )/*&& ds *n <=0*/)
               ds = adjustTrajectory(t0);
           else
               ds= this->computeTrajectory(t0);

            this->translateParent(Mi*ds);
            this->curr_t_in_dt =t;
            //update physics

            auto F = this->externalForces();
            auto c = t0.count();
            auto a = F*c;
            this->velocity += a;
//        }

    }

    void collision_controller::detectCollisions(double dt){

        bool movingObject =false;

        for(auto& sphere : _dynamic_spheres){
            if (sphere->state != states::Still){
                movingObject = true;
                break;
            }
        }

        if (movingObject){

        //loop for collision between dynamic objects (only spheres for now)
        for (auto it1 = _dynamic_spheres.begin() ; it1 != _dynamic_spheres.end() ; ++it1){
            if((*it1)->state != states::Still){
                for (auto it2 = it1+1 ; it2 != _dynamic_spheres.end() ; ++it2 ){
                    auto col = collision::detectCollision(**it1,**it2,seconds_type(dt));
                    const auto &sphere1= *it1;
                    const auto &sphere2= *it2;
                    auto min_ctidt = std::max(sphere1->curr_t_in_dt, sphere2->curr_t_in_dt);
                    if (col.flag == CollisionStateFlag::Collision && col.time < seconds_type(dt) && col.time >=min_ctidt ){
                        _collisions.emplace(col.time,CollisionObject(sphere1,sphere2,col.time));
                    }
                }
            }
        }

        //loop for collision with static objects (only dynamic spheres with static planes for now)
        for (auto &it1 : _dynamic_spheres){
            if( it1->state != states::Still){
                for (auto &it2 : _static_planes){
                    auto col = collision::detectCollision(*it1,*it2,seconds_type(dt));
                    if (col.flag == CollisionStateFlag::Collision && col.time < seconds_type(dt) && col.time >= it1->curr_t_in_dt){
                        _collisions.emplace(col.time,CollisionObject(it1,it2,col.time));
                    }
                }
            }
        }
  }
    }

    std::vector<StaticPPlane*> const collision_controller::getAttachedPlanes(DynamicPSphere* sphere) {

        return (_attachedPlanes[sphere]);

    }

    void collision_controller::attachPlane(DynamicPSphere *sphere, StaticPPlane *plane){

        _attachedPlanes[sphere].push_back(plane);
    }

    void collision_controller::detachPlane(DynamicPSphere*  sphere  , StaticPPlane* plane){
        std::swap(plane,_attachedPlanes[sphere].back());
        _attachedPlanes[sphere].pop_back();

    }

    void  collision_controller::detectStateChange( double dt){

        for (auto &sphere : _dynamic_spheres){

            std::vector<StaticPPlane*> p;
            states state;
            auto r = sphere->getRadius();
            auto pos = sphere->getMatrixToScene() * sphere->getPos();

            auto epsilon = 1e-5;
            auto dts = seconds_type(dt);
            auto min_dt = sphere->curr_t_in_dt;
            auto new_dt = dts - min_dt;
            auto ds = sphere->computeTrajectory(new_dt);
            auto planes = this->getAttachedPlanes(sphere) ;
            GMlib::APoint<float,3> q;
            GMlib::Vector <float,3>n {0.0f,0.0f,0.0f};
            double x ;

             if (planes.empty()){ //not attached
                 std::multimap<seconds_type,collision::CollisionObject>    possibleAttachedPlanes;
                 seconds_type  col_time = seconds_type(0.0);

                 for (auto &it2 : _static_planes){
                     auto col = collision::detectCollision(*sphere,*it2,seconds_type(dt));
                     if (col.flag == CollisionStateFlag::SingularityParallelAndTouching  && col.time < seconds_type(dt) &&  col.time >= sphere->curr_t_in_dt){
                         possibleAttachedPlanes.emplace(col.time,CollisionObject(sphere,it2,col.time));

                     }
                 }
                 if (!possibleAttachedPlanes.empty()){
                     //The sphere will become attached
                    //for now I just suppose it becomes still ( will deal with rolling later)

                     auto col = possibleAttachedPlanes.begin();
                     auto col_obj2 = col->second.obj2;
                     auto plane = dynamic_cast<StaticPPlane*> (col_obj2);
                     p.push_back(plane);
                     state =states::Still ;
                     col_time =  col->first;
                     _stateChanges.emplace(col_time,stateChangeObject(sphere, p, state,col_time));
                 }
             }

             else{ //attached
                 for (auto &it :planes){
                     auto M = it->evaluateParent(0.5f,0.5f,1,1);
                     auto pos= M(0)(0);
                     auto u = M(1)(0);
                     auto v = M(0)(1);
                     auto normal = GMlib::Vector<float,3>(u ^ v);
                     n+=normal;
                     q=pos;
                }
                n= GMlib::Vector <float,3>(n/planes.size()).getNormalized();

                auto d = (q + r * n) - pos;
                auto bla=std::abs(((-n*r) * ds) -(ds*ds));
                auto dsn= ds * n;
                auto dn= d*n;
                auto x = (d * n) / (ds * n);
                if (ds == 0.0){
                    x = min_dt.count();
                }

                if (sphere->state == states::Rolling){
                    if (ds * n > 0){
                        //std::cout<<"state changes from Rolling to Free";
                        state=states::Free;
                        //detachement
                       // _attachedPlanes[sphere].clear(); //empty vector
                        //return (stateChangeObject(sphere, states::Free));
                        //return stateChangeObject(sphere, p,state,x*new_dt +min_dt);
                        _stateChanges.emplace(x*new_dt +min_dt,stateChangeObject(sphere, planes, state,x*new_dt +min_dt));
                    }
                    else if (std::abs(((-n*r) * ds) -(ds*ds)) < epsilon){
                         //std::cout<<"state changes from Rolling to Still";
                         state=states::Still;
                        //return (stateChangeObject(sphere,states::Still)) ;
                         //return stateChangeObject(sphere, planes,state,x*new_dt +min_dt);
                         _stateChanges.emplace(x*new_dt +min_dt,stateChangeObject(sphere, planes, state,x*new_dt +min_dt));
                    }
                }

                else if (sphere->state == states::Still){
                    if (std::abs(((-n*r) * ds) -(ds*ds)) > epsilon){
                        //Correct trajectory
                        state=states::Rolling;
                        //std::cout<<"state changes from still to Rolling";
                        //return (stateChangeObject(sphere,  states::Rolling));
                        //return stateChangeObject(sphere, planes,state,x*new_dt +min_dt);
                        _stateChanges.emplace(x*new_dt +min_dt,stateChangeObject(sphere, planes, state,x*new_dt +min_dt));
                    }
                    else if (ds * n > 0  ){
                        //std::cout<<"state changes from still to Free";
                        //detachement
                        //_attachedPlanes[sphere].clear(); //empty vector
                       // return (stateChangeObject(sphere,  states::Free));
                        state=states::Free;
                        //return stateChangeObject(sphere, p,state,x*new_dt +min_dt);
                        _stateChanges.emplace(x*new_dt +min_dt,stateChangeObject(sphere, planes, state,x*new_dt +min_dt));
                    }
                }
             }




        }

    }


    void collision_controller::add (DynamicPSphere* const sphere) {
        _dynamic_spheres.push_back(sphere);
        _attachedPlanes[sphere];
        sphere->environment = &_environment;
    }

    GMlib::Vector<double,3>  DynamicPhysObject<GMlib::PSphere<float> >::adjustTrajectory(seconds_type dt){

        auto r = this->getRadius();
        auto s = this->getMatrixToScene() * this->getPos();
        auto ds = this->computeTrajectory(dt);
        auto p = s+ds;
        auto planes = sphereController->getAttachedPlanes(this) ;
        GMlib::Vector <float,3>n {0.0f,0.0f,0.0f};

        for (auto &it :planes){
            auto M = it->evaluateParent(0.5f,0.5f,1,1);
            auto q = M(0)(0);
            auto u = M(1)(0);
            auto v = M(0)(1);
            auto normal = GMlib::Vector<float,3>(u ^ v);
            n+=normal;
       }
          n= GMlib::Vector <float,3>(n/planes.size()).getNormalized();
          auto closest = this->sphereController->closestPoint(this,dt);
          auto d = n*r  +  closest;
          auto dsAdjusted = ds+d;

       return dsAdjusted;

    }

    GMlib::Vector<float,3>  collision_controller::closestPoint(DynamicPSphere*  sphere , seconds_type dt){

            //going to return   q-p  to  adjustTrajectory method

            auto max_dt = dt;
            auto min_dt = sphere->curr_t_in_dt;
            auto new_dt = max_dt -min_dt;
            auto epsilon= 1e-5;
            float u = 0.5;
            float v = 0.5;
            float delta_u = 0.5;
            float delta_v = 0.5;
            auto  s = sphere->getMatrixToScene() * sphere->getPos();
            auto r = sphere->getRadius();
            GMlib::Vector<double, 3> ds = sphere->computeTrajectory(new_dt);
            auto p = s+ds;
            GMlib::SqMatrix<float,2> A;
            GMlib::Vector<float, 2> b;
            GMlib::Vector<float,3> d{0.0f,0.0f,0.0f};


            auto  planes = _attachedPlanes[sphere];


            //iteration

            for ( int i=0; i<10;i++/* delta_u > epsilon && delta_v > epsilon*/){
            GMlib::Vector <float,3>Sn {0.0f,0.0f,0.0f};
            GMlib::Vector <float,3>Su {0.0f,0.0f,0.0f};
            GMlib::Vector <float,3>Sv {0.0f,0.0f,0.0f};
            GMlib::Vector <float,3>Suu {0.0f,0.0f,0.0f};
            GMlib::Vector <float,3>Svv {0.0f,0.0f,0.0f};
            GMlib::Vector <float,3>Suv {0.0f,0.0f,0.0f};
            GMlib::APoint <float,3>q{0.0};

            for(int it=0; it < planes.size(); it++){
                GMlib::Vector<float,3> normal{0.0f,0.0f,0.0f};
                GMlib::DMatrix<GMlib::Vector<float,3>>  M = planes[it]->evaluateParent(u,v,2,2);
                 q     = M(0)(0);
                 Su = M(1)(0);
                 Sv = M(0)(1);
                Suu = M(2)(0);
                Svv = M(0)(2);
                Suv = M(1)(1);
           }
            d =(q-p) * Sn *Sn;
            A[0][0] = d* Suu + Su * Su;
            A[0][1] = d* Suv + Su * Sv;
            A[1][0] = d* Suv + Su * Sv;
            A[1][1] = d* Svv + Sv * Sv;
            GMlib::SqMatrix<float,2> A_inv = A;
            A_inv.invert();
            b[0] = - d * Su;
            b[1] = - d * Sv;
            GMlib::APoint<float, 3> X = A_inv*b;
            delta_u = X(0);
            delta_v = X(1);
            u += delta_u;
            v += delta_v;


        }
            return d;
    }


} // END namespace collision

