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
           this->curr_t_in_dt =t;
       }


       GMlib::Vector<double,3> computeTrajectory( seconds_type dt) const override { //m

           auto const tay = 0.5 * this->externalForces() * dt.count()* dt.count(); //taylor
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

       std::sort(container.begin(),container.end(), [] (const auto & a , const auto & b) {
       return a.t_in_dt <b.t_in_dt;
   });

   //make unique;
   }

   //std::unique_ptr<Controller> unittestCollisionControllerFactory(){ return std::make_unique<MyController> (); }
   }
   // END namespace collision
#endif //COLLISION_LIBRARY_H
