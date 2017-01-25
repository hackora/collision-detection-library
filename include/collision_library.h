#ifndef COLLISION_LIBRARY_H
#define COLLISION_LIBRARY_H

// collision library interface
#include <collision_interface.h>



   namespace collision
   {

   class MyController : public Controller {
       GM_SCENEOBJECT (MyController)
   public:

       explicit MyController () = default;

       void add (DynamicPSphere* const sphere) { _dynamic_spheres.push_back(sphere); }
       void add (StaticPSphere* const sphere) { _static_spheres.push_back(sphere); }
       void add (StaticPPlane* const plane) { _static_planes.push_back(plane); }
       void add (StaticPCylinder* const cylinder) { _static_cylinders.push_back(cylinder); }
       void add (StaticPBezierSurf* const surf) { _static_bezier_surf.push_back(surf); }

   protected:
       void localSimulate (double dt) override;

       std::vector<DynamicPSphere*>    _dynamic_spheres;
       std::vector<StaticPSphere*>     _static_spheres;
       std::vector<StaticPPlane*>      _static_planes;
       std::vector<StaticPCylinder*>   _static_cylinders;
       std::vector<StaticPBezierSurf*> _static_bezier_surf;

       std::vector<collision::CollisionObject> _collisions;

   };

   template <>
   class DynamicPhysObject<GMlib::PSphere<float>> : public DynamicPhysObject_Base<GMlib::PSphere<float>> {
   public:
       using DynamicPhysObject_Base<GMlib::PSphere<float>>::DynamicPhysObject_Base;

       void    simulateToTInDt( seconds_type t ) override {

           auto dt = t - this->curr_t_in_dt;
           auto Mi = this->getMatrixToSceneInverse();
           //move

           auto ds = this->computeTrajectory(dt);
           this->translateParent(Mi*ds);
           this->curr_t_in_dt =t;
           //update physics
           auto F = this->externalForces();
           auto m = this->mass;
           auto a = 0.5*F*m*dt.count()*dt.count();
           this->velocity += a;


       }


       GMlib::Vector<double,3> computeTrajectory( seconds_type dt) const override { //m

           auto const tay = 0.5 * this->externalForces() * this->mass * dt.count()* dt.count(); //taylor
           return this->velocity * dt.count() + tay  ;
       }


       GMlib::Vector<double, 3> externalForces () const override ; // [m / s^2]
   };

   template <class PSurf_T, typename... Arguments>
   std::unique_ptr<DynamicPhysObject<PSurf_T>> unittestDynamicPhysObjectFactory(Arguments... parameters) {

       return std::make_unique<DynamicPhysObject<PSurf_T>> (parameters...);

   }

   template <class PSurf_T, typename... Arguments>
   std::unique_ptr<StaticPhysObject<PSurf_T>> unittestStaticPhysObjectFactory(Arguments... parameters) {

       return std::make_unique<StaticPhysObject<PSurf_T>> (parameters...);
   }

   template <class Container_T > void sortAndMakeUnique( Container_T& container) {

       /*std::sort(container.begin(),container.end(), [] (const auto & a , const auto & b) {
       return a->second.t_in_dt <b->second.t_in_dt;

   });*/

   //make unique;
       for (auto it1 = container.begin() ; it1 != container.end() ; ++it1){
           for (auto it2 = std::next(it1,1) ; it2 != container.end() ;){
               if (it2->second.obj1 == it1->second.obj1 || it2->second.obj2 == it1->second.obj1 ||
                       it2->second.obj1 == it1->second.obj2 ||
                       (it2->second.obj2 == it1->second.obj2 && dynamic_cast<DynamicPSphere*> (it1->second.obj2))) //I suppose that obj1 in collisionObject is always dynamic
                     {
                       container.erase(it2++);
                     }
               else
                     {
                       ++it2;
                     }
           }
       }



   }

   //std::unique_ptr<Controller> unittestCollisionControllerFactory(){ return std::make_unique<MyController> (); }
   }
   // END namespace collision
#endif //COLLISION_LIBRARY_H
