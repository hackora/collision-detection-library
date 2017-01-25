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
