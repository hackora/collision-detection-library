// gtest
#include <gtest/gtest.h> // googletest header file

// collision library
#include <collision_library.h>
using namespace collision;

// gmlib
#include <gmParametricsModule>
using namespace GMlib;

// stl
#include <memory>
#include <chrono>
#include <iostream>
#include <map>
#include <typeinfo>
using namespace std::chrono_literals;


TEST(MyUniqueTestCategory, MyCategory_UniqueTestName_WhichPasses) {

  EXPECT_TRUE(true);
}

TEST(MyUniqueTestCategory, MyCategory_UniqueTestName_WhichFails) {

  EXPECT_FALSE(false);
}

TEST(SortAndMakeUnique, Sorting) {

  auto sphere1 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere2 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere3 = unittestDynamicPhysObjectFactory<PSphere<float>>();

  auto cylinder1 = unittestStaticPhysObjectFactory<PCylinder<float>>();

  std::multimap<seconds_type,CollisionObject> collisions;

  collisions.emplace(seconds_type{0.3ms},CollisionObject(sphere2.get(), cylinder1.get(), 0.3ms));
  collisions.emplace(seconds_type{0.1ms},CollisionObject(sphere1.get(), cylinder1.get(), 0.1ms ));
  collisions.emplace(seconds_type{0.2ms}, CollisionObject(sphere3.get(), cylinder1.get(), 0.2ms ));

/*  std::cout << "mymultimap contains:";
    for (auto& it: collisions)
      std::cout << " [" << it.first << ']';
    std::cout << '\n';
*/
  CollisionObject c0( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c1( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c2( sphere1.get(), cylinder1.get(), 0.1ms );

  //sortAndMakeUnique(collisions);

  int i=0;
  for (auto& it: collisions){
      if (i==0){
          c0 = it.second;
          i++;
      }
      else if (i ==1){
          c1 = it.second;
          i++;
      }
      else
          c2 = it.second;
  }

  EXPECT_EQ(c0.obj1, sphere1.get()); EXPECT_EQ( c0.t_in_dt, seconds_type{0.1ms} );
  EXPECT_EQ(c1.obj1, sphere3.get()); EXPECT_EQ( c1.t_in_dt, seconds_type{0.2ms} );
  EXPECT_EQ(c2.obj1, sphere2.get()); EXPECT_EQ( c2.t_in_dt, seconds_type{0.3ms} );
}

TEST(SortAndMakeUnique, MakeUnique) {

  auto sphere1 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere2 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere3 = unittestDynamicPhysObjectFactory<PSphere<float>>();

  auto cylinder1 = unittestStaticPhysObjectFactory<PCylinder<float>>();
  auto cylinder2 = unittestStaticPhysObjectFactory<PCylinder<float>>();

  std::cout << "types of sphere:";
         std::cout << " [" << typeid(sphere1.get()).name() << ']';
      std::cout << '\n';
  std::cout << "types of cylinder:";
         std::cout << " [" << typeid(cylinder1.get()).name() << ']';
      std::cout << '\n';

  std::multimap<seconds_type,CollisionObject> collisions;

  collisions.emplace(seconds_type{0.4ms},CollisionObject(sphere2.get(), cylinder1.get(), 0.4ms));
  collisions.emplace(seconds_type{0.1ms},CollisionObject(sphere1.get(), cylinder1.get(), 0.1ms ));
  collisions.emplace(seconds_type{0.2ms}, CollisionObject(sphere3.get(), cylinder2.get(), 0.2ms ));
  collisions.emplace(seconds_type{0.3ms}, CollisionObject(sphere3.get(), sphere2.get(), 0.3ms ));

/*  std::cout << "mymultimap contains:";
    for (auto& it: collisions)
      std::cout << " [" << it.first << ']';
    std::cout << '\n';
*/
  CollisionObject c0( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c1( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c2( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c3( sphere1.get(), cylinder1.get(), 0.1ms );

  //sortAndMakeUnique(collisions);

  std::cout << "mymultimap contains:";
      for (auto& it: collisions)
        std::cout << " [" << it.first.count() << ']';
      std::cout << '\n';

     if(dynamic_cast<DynamicPSphere*> (c0.obj1))
         std::cout << " [" << typeid(c0.obj1).name() << '\n';


      std::cout << "types of sphere:";
             std::cout << " [" << typeid(c0.obj1).name() << ']';
          std::cout << '\n';
      std::cout << "types of cylinder:";
             std::cout << " [" << typeid(c0.obj2).name() << ']';
          std::cout << '\n';

  sortAndMakeUnique(collisions);

  std::cout << "After make unique mymultimap contains:";
      for (auto& it: collisions)
        std::cout << " [" << it.first.count() << ']';
      std::cout << '\n';

      int i=0;
      for (auto& it: collisions){
          if (i==0){
              c0 = it.second;
              i++;
          }
          else if (i ==1){
              c1 = it.second;
              i++;
          }
          else {
              c2 = it.second;
              i++;
          }

      }


  EXPECT_EQ(c0.obj1, sphere1.get()); EXPECT_EQ( c0.t_in_dt, seconds_type{0.1ms} );
  EXPECT_EQ(c1.obj1, sphere3.get()); EXPECT_EQ( c1.t_in_dt, seconds_type{0.2ms} );
  EXPECT_EQ(c2.obj1, sphere2.get()); EXPECT_EQ( c2.t_in_dt, seconds_type{0.4ms} );
}



//adapted from interface tests



TEST(SortAndMakeUnique, CompareFirstToFirst1) {

  auto sphere1 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere2 = unittestDynamicPhysObjectFactory<PSphere<float>>();

  auto cylinder1 = unittestStaticPhysObjectFactory<PCylinder<float>>();

  std::multimap<seconds_type,CollisionObject> collisions;

  collisions.emplace(seconds_type{0.1ms},CollisionObject(sphere1.get(), cylinder1.get(), 0.1ms ));
  collisions.emplace(seconds_type{0.2ms}, CollisionObject(sphere2.get(), cylinder1.get(), 0.2ms ));
  collisions.emplace(seconds_type{0.3ms}, CollisionObject(sphere1.get(), cylinder1.get(), 0.3ms ));

  sortAndMakeUnique(collisions);

  int i=0;
  CollisionObject c0( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c1( sphere1.get(), cylinder1.get(), 0.1ms );
  for (auto& it: collisions){
      if (i==0){
          c0 = it.second;
          i++;
      }
      else {
          c1 = it.second;
      }
  }

  EXPECT_EQ(collisions.size(), 2);
  EXPECT_EQ(c0.obj1, sphere1.get());
  EXPECT_EQ(c1.obj1, sphere2.get());
}

TEST(SortAndMakeUnique, CompareFirstToSecond1) {

  auto sphere1 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere2 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere3 = unittestDynamicPhysObjectFactory<PSphere<float>>();

  auto cylinder1 = unittestStaticPhysObjectFactory<PCylinder<float>>();

  std::multimap<seconds_type,CollisionObject> collisions;

  collisions.emplace(seconds_type{0.1ms},CollisionObject(sphere1.get(), cylinder1.get(), 0.1ms ));
  collisions.emplace(seconds_type{0.2ms}, CollisionObject(sphere2.get(), cylinder1.get(), 0.2ms ));
  collisions.emplace(seconds_type{0.3ms}, CollisionObject(sphere2.get(), sphere1.get(), 0.3ms ));

  sortAndMakeUnique(collisions);
  int i=0;
  CollisionObject c0( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c1( sphere1.get(), cylinder1.get(), 0.1ms );
  for (auto& it: collisions){
      if (i==0){
          c0 = it.second;
          i++;
      }
      else {
          c1 = it.second;
      }
  }


  EXPECT_EQ(collisions.size(), 2);
  EXPECT_EQ(c0.obj1, sphere1.get());
  EXPECT_EQ(c1.obj1, sphere2.get());

}

TEST(SortAndMakeUnique, CompareSecondToFirst1) {

  auto sphere1 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere2 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere3 = unittestDynamicPhysObjectFactory<PSphere<float>>();

  auto cylinder1 = unittestStaticPhysObjectFactory<PCylinder<float>>();

  std::multimap<seconds_type,CollisionObject> collisions;

  collisions.emplace(seconds_type{0.1ms},CollisionObject(sphere1.get(), sphere2.get(), 0.1ms ));
  collisions.emplace(seconds_type{0.2ms}, CollisionObject(sphere3.get(), cylinder1.get(), 0.2ms ));
  collisions.emplace(seconds_type{0.3ms}, CollisionObject(sphere2.get(), cylinder1.get(), 0.3ms ));

  sortAndMakeUnique(collisions);

  int i=0;
  CollisionObject c0( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c1( sphere1.get(), cylinder1.get(), 0.1ms );
  for (auto& it: collisions){
      if (i==0){
          c0 = it.second;
          i++;
      }
      else {
          c1 = it.second;
      }
  }

  EXPECT_EQ(collisions.size(), 2);
  EXPECT_EQ(c0.obj1, sphere1.get());
  EXPECT_EQ(c1.obj1, sphere3.get());

}

TEST(SortAndMakeUnique, CompareSecondToSecond) {

  auto sphere1 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere2 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere3 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere4 = unittestDynamicPhysObjectFactory<PSphere<float>>();

  auto cylinder1 = unittestStaticPhysObjectFactory<PCylinder<float>>();

  std::multimap<seconds_type,CollisionObject> collisions;

  collisions.emplace(seconds_type{0.1ms},CollisionObject(sphere1.get(), sphere2.get(), 0.1ms ));
  collisions.emplace(seconds_type{0.2ms}, CollisionObject(sphere3.get(), cylinder1.get(), 0.2ms ));
  collisions.emplace(seconds_type{0.3ms}, CollisionObject(sphere4.get(), sphere2.get(), 0.3ms ));

  sortAndMakeUnique(collisions);

  int i=0;
  CollisionObject c0( sphere1.get(), cylinder1.get(), 0.1ms );
  CollisionObject c1( sphere1.get(), cylinder1.get(), 0.1ms );
  for (auto& it: collisions){
      if (i==0){
          c0 = it.second;
          i++;
      }
      else {
          c1 = it.second;
      }
  }

  EXPECT_EQ(collisions.size(), 2);
  EXPECT_EQ(c0.obj1, sphere1.get());
  EXPECT_EQ(c1.obj1, sphere3.get());

}

TEST(SortAndMakeUnique, ComplexExample) {


  auto sphere1 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere2 = unittestDynamicPhysObjectFactory<PSphere<float>>();
  auto sphere3 = unittestDynamicPhysObjectFactory<PSphere<float>>();

  auto cylinder = unittestStaticPhysObjectFactory<PCylinder<float>>();

  std::multimap<seconds_type,CollisionObject> collisions;

  collisions.emplace(seconds_type{0.35ms},CollisionObject(sphere1.get(), cylinder.get(), 0.35ms ));
  collisions.emplace(seconds_type{0.3ms}, CollisionObject(sphere2.get(), cylinder.get(), 0.3ms ));
  collisions.emplace(seconds_type{0.1ms}, CollisionObject(sphere1.get(), sphere3.get(), 0.1ms ));
  collisions.emplace(seconds_type{0.11ms},CollisionObject(sphere3.get(), sphere1.get(), 0.11ms ));
  collisions.emplace(seconds_type{0.41ms}, CollisionObject(sphere2.get(), sphere3.get(), 0.41ms ));
  collisions.emplace(seconds_type{0.4ms}, CollisionObject(sphere1.get(), sphere2.get(), 0.4ms ));

  sortAndMakeUnique(collisions);

  int i=0;
  CollisionObject c0( sphere1.get(), cylinder.get(), 0.1ms );
  CollisionObject c1( sphere1.get(), cylinder.get(), 0.1ms );

  for (auto& it: collisions){
      if (i==0){
          c0 = it.second;
          i++;
      }
      else {
          c1 = it.second;
      }
  }

  EXPECT_EQ(c0.obj1, sphere1.get()); EXPECT_EQ(c0.obj2, sphere3.get()); EXPECT_EQ( c0.t_in_dt, seconds_type{0.1ms} );
  EXPECT_EQ(c1.obj1, sphere2.get()); EXPECT_EQ(c1.obj2, cylinder.get()); EXPECT_EQ( c1.t_in_dt, seconds_type{0.3ms} );
}


TEST(DSphereDSphereCollisionDetection, SimulationController) {

    Environment env;

    std::vector<std::unique_ptr<DynamicPSphere>> spheres;
    spheres.reserve(2);

    spheres.emplace_back(unittestDynamicPhysObjectFactory<GMlib::PSphere<float>>());
    spheres.back()->environment = &env;
    spheres.back()->velocity = Vector<double, 3>{10.0, 10.1, 0.0};
    spheres.back()->curr_t_in_dt = seconds_type{0};

    spheres.emplace_back(unittestDynamicPhysObjectFactory<GMlib::PSphere<float>>());
    spheres.back()->environment = &env;
    spheres.back()->velocity = Vector<double, 3>{-2.0, 1.1, 0.0};
    spheres.back()->curr_t_in_dt = seconds_type{0};


    std::vector<std::unique_ptr<StaticPPlane>> planes;
    planes.reserve (4);
    planes.emplace_back (unittestStaticPhysObjectFactory<GMlib::PPlane<float>> (
        GMlib::Point<float, 3> (-10.0f, -10.0f, 5.0f),
        GMlib::Vector<float, 3> (20.0f, 0.0f, 0.0f),
        GMlib::Vector<float, 3> (0.0f, 0.0f, -5.0f)));


    planes.emplace_back (unittestStaticPhysObjectFactory<GMlib::PPlane<float>> (
        GMlib::Point<float, 3> (10.0f, -10.0f, 5.0f),
        GMlib::Vector<float, 3> (0.0f, 20.0f, 0.0f),
        GMlib::Vector<float, 3> (0.0f, 0.0f, -5.0f)));

    planes.emplace_back (unittestStaticPhysObjectFactory<GMlib::PPlane<float>> (
        GMlib::Point<float, 3> (10.0f, 10.0f, 5.0f),
        GMlib::Vector<float, 3> (-20.0f, 0.0f, 0.0f),
        GMlib::Vector<float, 3> (0.0f, 0.0f, -5.0f)));

    planes.emplace_back (unittestStaticPhysObjectFactory<GMlib::PPlane<float>> (
        GMlib::Point<float, 3> (-10.0f, 10.0f, 5.0f),
        GMlib::Vector<float, 3> (0.0f, -20.0f, 0.0f),
        GMlib::Vector<float, 3> (0.0f, 0.0f, -5.0f)));

    auto controller = unittestCollisionControllerFactory();
    for(auto& sphere : spheres)
      controller->add(sphere.get());
    for(auto& plane : planes)
      controller->add(plane.get());


    Scene scene;
    scene.insert(controller.get());
    scene.prepare();

    scene.enabledFixedDt();
    scene.setFixedDt(1.0);
    scene.start();
    scene.simulate();
    scene.prepare();

}
