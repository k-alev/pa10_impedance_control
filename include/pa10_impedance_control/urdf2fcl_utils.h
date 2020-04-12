#pragma once

#include <urdf/model.h>
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include <kdl/chain.hpp>
#include <memory>

namespace urdf2fcl
{

std::shared_ptr<fcl::CollisionGeometry> getLinkGeometry(const urdf::GeometrySharedPtr& geomPtr)
{
    if(geomPtr->type==geomPtr->BOX)
    {
        std::cout<<"BOX"<<std::endl;
        boost::shared_ptr<urdf::Box>  handle = boost::dynamic_pointer_cast<urdf::Box>(geomPtr);
        auto dim = handle->dim;
        fcl::Box *box = new fcl::Box(dim.x, dim.y, dim.z);
        return std::shared_ptr<fcl::CollisionGeometry>(box);
    }
    if(geomPtr->type==geomPtr->CYLINDER)
    {
        std::cout<<"CYLINDER"<<std::endl;
        boost::shared_ptr<urdf::Cylinder>  handle = boost::dynamic_pointer_cast<urdf::Cylinder>(geomPtr);
        auto radius = handle->radius;
        auto length = handle->length;
        fcl::Cylinder *cylinder = new fcl::Cylinder(radius, length);
        return std::shared_ptr<fcl::CollisionGeometry>(cylinder);
    }
    if(geomPtr->type==geomPtr->MESH)
    {
        throw(std::runtime_error("Collision geometry is a MESH. MESH support is not implemented yet"));
    }
    if(geomPtr->type==geomPtr->SPHERE)
    {
        std::cout<<"SPHERE"<<std::endl;
        boost::shared_ptr<urdf::Sphere>  handle = boost::dynamic_pointer_cast<urdf::Sphere>(geomPtr);
        auto radius = handle->radius;
        fcl::Sphere *sphere = new fcl::Sphere(radius);
        return std::shared_ptr<fcl::CollisionGeometry>(sphere);
    }
}

fcl::Transform3f getLinkTransform(urdf::Pose pose)
{
    auto pos = pose.position;
    double qx, qy, qz, qw;
    pose.rotation.getQuaternion(qx, qy, qz, qw);

    fcl::Vec3f T = fcl::Vec3f(pos.x, pos.y, pos.z);
    fcl::Quaternion3f Q = fcl::Quaternion3f(qw, qx, qy, qz);
    fcl::Transform3f transform(Q, T);
    return transform;

}

void getCollisionObject(const urdf::LinkSharedPtr& linkPtr, std::vector<fcl::CollisionObject*> objs)
{
    
    if ((linkPtr->collision != nullptr))
    {
        std::cout << "Name: " << linkPtr->name << std::endl;

        // get link geometry (return fcl)
        auto geomFcl = getLinkGeometry(linkPtr->collision->geometry);

        // std::cout << "FCL_Type: " <<geomFcl->getNodeType() << std::endl;

        // get link transformation (translation, rotation) (return fcl)
        auto tfmFcl = getLinkTransform(linkPtr->collision->origin);

        // fcl::CollisionObject* obj = new fcl::CollisionObject(geomFcl, tfmFcl);
        objs.push_back(new fcl::CollisionObject(geomFcl, tfmFcl));
    }  
}

std::vector<fcl::CollisionObject*> getCollisionObjects(const std::string& desc) //this should be for the env so should have arg an std::vector<std::string> named as getEnvCollisionObjs
{
    urdf::Model model;
    if (!model.initString(desc))
        throw std::runtime_error("Failed to parse URDF string.");

    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);
    std::vector<fcl::CollisionObject*> objs;

    for (auto it = links.begin(); it != links.end(); ++it)
    {
        getCollisionObject(*it, objs);
    }
}

std::vector<fcl::CollisionObject*>  getCollisionObjects(const std::string& desc, const KDL::Chain& chain) //this should be named as getRobotCollisionObjs
{
    urdf::Model model;
    if (!model.initString(desc))
        throw std::runtime_error("Failed to parse URDF string.");

    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);
    std::vector<fcl::CollisionObject*> objs;
    auto segs = chain.segments;

    
    for (auto seg_it = segs.begin(); seg_it != segs.end(); ++seg_it)
    {
        for (auto lnk_it = links.begin(); lnk_it != links.end(); ++lnk_it)
        {
            if(seg_it->getName() == (*lnk_it)->name)
            {
                getCollisionObject(*lnk_it, objs);
            }
        }
    }
}

} // namespace urdf2fcl
