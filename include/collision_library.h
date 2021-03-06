#ifndef COLLISION_LIBRARY_H
#define COLLISION_LIBRARY_H

// collision library interface
#include <collision_interface.h>
#include <unordered_map>



namespace collision
{
enum class states {
    Free,
    Rolling,
    Still
};

struct stateChangeObject{
    DynamicPSphere* obj;
    std::vector<StaticPPlane*> attachedPlanes;
    states stateChanges;
    seconds_type  t_in_dt;

    stateChangeObject( DynamicPSphere* o, std::vector<StaticPPlane*> p,  states s,  seconds_type  t )
        : obj{o},  attachedPlanes{p}, stateChanges{s},  t_in_dt{t}{}
};


class collision_controller : public Controller {
    GM_SCENEOBJECT (collision_controller)

    public:

        explicit collision_controller () = default;

    void add (DynamicPSphere* const sphere);
    void add (StaticPSphere* const sphere) { _static_spheres.push_back(sphere);  }
    void add (StaticPPlane* const plane) { _static_planes.push_back(plane); }
    void add (StaticPCylinder* const cylinder) { _static_cylinders.push_back(cylinder); }
    void add (StaticPBezierSurf* const surf) { _static_bezier_surf.push_back(surf); }
    std::vector<StaticPPlane*>  const getAttachedPlanes(DynamicPSphere* sphere) ;
    GMlib::Vector<float,3>  closestPoint(DynamicPSphere*  sphere , seconds_type dt);
    Environment    _noGravity;

protected:
    void localSimulate (double dt) override;
    void detectCollisions(double dt);
    void attachPlane(DynamicPSphere*  sphere  , StaticPPlane* plane);
    void detachPlane(DynamicPSphere*  sphere  , StaticPPlane* plane);
    void detectStateChanges(double dt);
    stateChangeObject detectStateChange( DynamicPSphere* sphere, double dt);


    std::vector<DynamicPSphere*>                                                                                             _dynamic_spheres;
    std::vector<StaticPSphere*>                                                                                                   _static_spheres;
    std::vector<StaticPPlane*>                                                                                                      _static_planes;
    std::vector<StaticPCylinder*>                                                                                                 _static_cylinders;
    std::vector<StaticPBezierSurf*>                                                                                             _static_bezier_surf;

    std::multimap<seconds_type,collision::CollisionObject>                                                _collisions;
    std::multimap<seconds_type,stateChangeObject>                                                          _stateChanges;
    std::unordered_map<DynamicPSphere* , std::vector<StaticPPlane*>>                    _attachedPlanes;
    DefaultEnvironment                                                                                                                    _environment;

};

template <>
class DynamicPhysObject<GMlib::PSphere<float>> : public DynamicPhysObject_Base<GMlib::PSphere<float>> {
public:
    using DynamicPhysObject_Base<GMlib::PSphere<float>>::DynamicPhysObject_Base;

    states                                                                        state=states::Free;

    collision_controller*                                             sphereController;


    void    simulateToTInDt( seconds_type t ) override;
    GMlib::Vector<double,3>  adjustTrajectory(seconds_type dt);
    GMlib::Vector<double,3> computeTrajectory( seconds_type dt) const override { //m

        auto t = dt.count();
        auto F = this->externalForces();
        auto const tay = 0.5  * F * t * t; //taylor
        return this->velocity * t + tay  ;

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


/**
 * Make unique method for the state change container
 */

template <class Container_T > void sortAndMakeUnique( Container_T& container) {

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

/**
 * Make unique method for the state change container
 */

template <class Container_T > void sortAndMakeUniqueState( Container_T& container) {

    for (auto it1 = container.begin() ; it1 != container.end() ; ++it1){
        for (auto it2 = std::next(it1,1) ; it2 != container.end() ;){
            if (it2->second.obj == it1->second.obj)
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

/**
 * Cross make unique of  the two containers ( Collision objects and State objects)
 */

template <class Container_T1, class Container_T2 > void sortAndMakeUnique( Container_T1& container1, Container_T2& container2){

    auto it1 = container1.begin() ;

    while (it1!= container1.end()) {
        auto it2 = container2.begin() ;
        while (it2!= container2.end()){
            if ((it1->second.obj1 == it2->second.obj) ||( it1->second.obj2 == it2->second.obj && dynamic_cast<DynamicPSphere*> (it1->second.obj2))){
                if (it1->first <= it2->first){
                    container2.erase(it2++);
                    ++it1;
                }
                else{
                    container1.erase(it1++);
                    if (! container1.empty())
                        it2 = container2.begin() ;
                    else
                        break;
                }
            }
        }
    }
}

}

// END namespace collision
#endif //COLLISION_LIBRARY_H
