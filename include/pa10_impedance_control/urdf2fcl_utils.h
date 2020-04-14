#pragma once

#include <urdf/model.h>
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include <kdl/chain.hpp>

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

void getCollisionObject(const urdf::LinkSharedPtr& linkPtr, std::pair<std::vector<fcl::CollisionObject*>, std::vector<fcl::Transform3f>>& result)
{
    
    if ((linkPtr->collision != nullptr))
    {
        std::cout << "Name: " << linkPtr->name << std::endl;
        auto geomFcl = getLinkGeometry(linkPtr->collision->geometry);
        auto offset = getLinkTransform(linkPtr->collision->origin);

        // offsets.push_back(offset);
        // objs.push_back(new fcl::CollisionObject(geomFcl, offset));
        result.second.push_back(offset);
        result.first.push_back(new fcl::CollisionObject(geomFcl, offset));
    }  
}

std::pair<std::vector<fcl::CollisionObject*>, std::vector<fcl::Transform3f>> getEnvCollisionObjects(const std::vector<std::string>& desc) //this should be for the env so should have arg an std::vector<std::string> named as getEnvCollisionObjs
{
    std::pair<std::vector<fcl::CollisionObject*>, std::vector<fcl::Transform3f>> result;
    for(auto it = desc.begin(); it!= desc.end(); ++it)
    {
        urdf::Model model;
        if (!model.initString(*it))
            throw std::runtime_error("Failed to parse URDF string.");

        std::vector<urdf::LinkSharedPtr> links;
        model.getLinks(links);
        getCollisionObject(links[0], result);
    }
    return result;
}

std::pair<std::vector<fcl::CollisionObject*>, std::vector<fcl::Transform3f>> getRobotCollisionObjects(const std::string& desc, const KDL::Chain& chain) 
{
    urdf::Model model;
    if (!model.initString(desc))
        throw std::runtime_error("Failed to parse URDF string.");

    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);
    std::pair<std::vector<fcl::CollisionObject*>, std::vector<fcl::Transform3f>> result;
    auto segs = chain.segments;

    for (auto seg_it = segs.begin(); seg_it != segs.end(); ++seg_it)
    {
        for (auto lnk_it = links.begin(); lnk_it != links.end(); ++lnk_it)
        {
            if(seg_it->getName() == (*lnk_it)->name)
            {
                getCollisionObject(*lnk_it, result);
            }
        }
    }
    return result;
}

} // namespace urdf2fcl
