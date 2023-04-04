/*
 * ColliderManagerFcl.tpp
 *
 *  Created on: Jul 18, 2017
 *      Author: Perry Franklin
 */

#include <collisions_fcl/ColliderManagerFcl.hpp>

#include <collisions/CollisionGroup.hpp>

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>

namespace collisions_fcl {

collisions::DistanceResults ColliderManagerFcl::checkDistance(const collisions::CollisionGroup& group1,
                                                              const collisions::CollisionGroup& group2,
                                                              const collisions::DistanceOptions& distance_options ){

  using DistancePair = typename collisions::DistanceResults::DistancePair;
  using Position = typename collisions::DistanceResults::Position;

  std::vector<  std::vector<fcl::CollisionObject<double> > > fcl_groups1;

  for (size_t i = 0; i < group1.size(); ++i){

    const collisions::CollisionBodyConstPtr& element = group1.getBodies()[i];

    // This is rather inefficient; lots of copying. TODO: Make betterer.
    std::vector< fcl::CollisionObject<double> > fcl_objects;
    if (FclConverter::FclObjectFromCollisionBody(*(element), 0.0, fcl_objects)){
      fcl_groups1.push_back( fcl_objects );
    }
  }

  std::vector< std::vector< fcl::CollisionObject<double> > > fcl_groups2;

  for (size_t i = 0; i < group2.size(); ++i){

    const collisions::CollisionBodyConstPtr& element = group2.getBodies()[i];

    std::vector< fcl::CollisionObject<double> > fcl_objects;
    if (FclConverter::FclObjectFromCollisionBody(*(element), 0.0, fcl_objects)){
      fcl_groups2.push_back( fcl_objects );
    }
  }

  collisions::DistanceResults results;

  for (size_t i = 0; i < fcl_groups1.size(); ++i){

    std::vector< fcl::CollisionObject<double> >& object_group1 = fcl_groups1[i];

    for (size_t j = 0; j < fcl_groups2.size(); ++j){

      std::vector< fcl::CollisionObject<double> >& object_group2 = fcl_groups2[j];

      double closest_distance_for_groups = std::numeric_limits<double>::max();
      fcl::Vector3<double> point_1;
      fcl::Vector3<double> point_2;

      for (fcl::CollisionObject<double>& object1 : object_group1){
        for (fcl::CollisionObject<double>& object2 : object_group2){

          fcl::DistanceRequest<double> fcl_request(true,true);

          fcl::DistanceResult<double> fcl_result;

          fcl::distance(&(object1), &(object2), fcl_request, fcl_result);

          if (fcl_result.min_distance < closest_distance_for_groups){
            closest_distance_for_groups = fcl_result.min_distance;
            point_1 = fcl_result.nearest_points[0];
            point_2 = fcl_result.nearest_points[1];
          }


        } // group2 object loop

      } // group1 object loop

      collisions::DistanceResult single_result;
      single_result.distance_pair = DistancePair(group1.getBodies()[i]->getId(), group2.getBodies()[j]->getId());
      single_result.distance = closest_distance_for_groups;

      std::cout<<"point1 "<<point_1<<std::endl;
      std::cout<<"point2 "<<point_2<<std::endl;
      single_result.nearest_point_1 = Position(point_1);
      single_result.nearest_point_2 = Position(point_2);

      results.getResults().emplace(std::make_pair(single_result.distance_pair, single_result));

    } // fcl_groups2 loop

  } // fcl_groups1 loop

  return results;

}


collisions::CollisionResults
  ColliderManagerFcl::checkCollision(const collisions::CollisionGroup& group1,
									 const collisions::CollisionGroup& group2,
                                     const collisions::CollisionOptions& collision_options){

  std::vector<  std::vector<fcl::CollisionObject<double> > > fcl_groups1;
  const std::vector<  collisions::CollisionBodyConstPtr >& group1_bodies = group1.getBodies();

  for (size_t i = 0; i < group1_bodies.size(); ++i){

    const collisions::CollisionBodyConstPtr& element = group1_bodies[i];

    // This is rather inefficient; lots of copying. TODO: Make betterer.
    std::vector< fcl::CollisionObject<double> > fcl_objects;
    if (FclConverter::FclObjectFromCollisionBody(*(element), collision_options.extra_margin, fcl_objects)){
      fcl_groups1.push_back( fcl_objects );
    }
  }

  std::vector< std::vector< fcl::CollisionObject<double> > > fcl_groups2;
  const std::vector<  collisions::CollisionBodyConstPtr >& group2_bodies = group2.getBodies();

  for (size_t i = 0; i < group2_bodies.size(); ++i){

    const collisions::CollisionBodyConstPtr& element = group2_bodies[i];

    std::vector< fcl::CollisionObject<double> > fcl_objects;
    if (FclConverter::FclObjectFromCollisionBody(*(element), collision_options.extra_margin, fcl_objects)){
      fcl_groups2.push_back( fcl_objects );
    }
  }

  typename collisions::CollisionResults results;

  for (size_t i = 0; i < fcl_groups1.size(); ++i){

    std::vector< fcl::CollisionObject<double> >& object_group1 = fcl_groups1[i];

    for (size_t j = 0; j < fcl_groups2.size(); ++j){

      std::vector< fcl::CollisionObject<double> >& object_group2 = fcl_groups2[j];

      // This Done flag is on a  per link basis, not for the whole process
      bool done = false;
      for (fcl::CollisionObject<double>& object1 : object_group1){
        for (fcl::CollisionObject<double>& object2 : object_group2){

          fcl::CollisionRequest<double> fcl_request;
          fcl_request.num_max_contacts = 10;
          fcl_request.enable_contact = 1;
          results.contacts_calculated = fcl_request.enable_contact;
          fcl::CollisionResult<double> fcl_result;
          fcl::collide(&(object1), &(object2), fcl_request, fcl_result);

          if (fcl_result.isCollision()){
            results.colliding_pairs.push_back(std::make_pair(i, j));
            done = true;

            std::vector<fcl::Contactd> contacts;
            fcl_result.getContacts(contacts);
            for (fcl::Contactd contact : contacts){
              results.contact_locations.push_back(contact.pos);
            }
          }
          if (done){
            break;
          }

        } // group2 object loop

        if (done){
          break;
        }

      } // group1 object loop

    } // fcl_groups2 loop

  } // fcl_groups1 loop

  return results;

}

}
