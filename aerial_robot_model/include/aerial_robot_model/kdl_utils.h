#pragma once

#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_kdl.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <vector>
#include <kdl/rotationalinertia.hpp>
#include <kdl/tree.hpp>

namespace aerial_robot_model {

  namespace {
    template <class T1, class T2, class Callback> std::vector<T1> convertVector(const std::vector<T2>& in, Callback callback)
    {
      std::vector<T1> out;
      out.reserve(in.size());
      for(const auto& elem : in)
        out.push_back(callback(elem));
      return out;
    }
  }

  inline bool isValidRotation(const KDL::Rotation& m)
  {
    double x, y, z, w;
    m.GetQuaternion(x, y, z, w);
    if(std::fabs(1 - Eigen::Quaterniond(w, x, y, z).squaredNorm())  < 1e-6) return true;
    else return false;
  }

  inline geometry_msgs::TransformStamped kdlToMsg(const KDL::Frame& in)
  {
    return tf2::kdlToTransform(in);
  }

  inline geometry_msgs::PointStamped kdlToMsg(const KDL::Vector& in)
  {
    tf2::Stamped<KDL::Vector> tmp;
    tmp.setData(in);
    geometry_msgs::PointStamped out;
    tf2::convert(tmp, out);
    return out;
  }

  inline std::vector<geometry_msgs::PointStamped> kdlToMsg(const std::vector<KDL::Vector>& in)
  {
    return convertVector<geometry_msgs::PointStamped, KDL::Vector>(in,
                                                                   [](const KDL::Vector& in)->geometry_msgs::PointStamped {
                                                                     return kdlToMsg(in);
                                                                   });
  }

  inline Eigen::Affine3d kdlToEigen(const KDL::Frame& in)
  {
    Eigen::Affine3d out;
    tf::transformKDLToEigen(in, out);
    return out;
  }

  inline Eigen::Vector3d kdlToEigen(const KDL::Vector& in)
  {
    Eigen::Vector3d out;
    tf::vectorKDLToEigen(in, out);
    return out;
  }

  inline Eigen::Matrix3d kdlToEigen(const KDL::RotationalInertia& in)
  {
    return Eigen::Map<const Eigen::Matrix3d>(in.data);
  }

  inline Eigen::Matrix3d kdlToEigen(const KDL::Rotation& in)
  {
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(in.data);
  }

  inline std::vector<Eigen::Vector3d> kdlToEigen(const std::vector<KDL::Vector>& in)
  {
    return convertVector<Eigen::Vector3d, KDL::Vector>(in,
                                                       [](const KDL::Vector& in)->Eigen::Vector3d {
                                                         return kdlToEigen(in);
                                                       });
  }

  inline std::vector<Eigen::Matrix3d> kdlToEigen(const std::vector<KDL::Rotation>& in)
  {
    return convertVector<Eigen::Matrix3d, KDL::Rotation>(in,
                                                         [](const KDL::Rotation& in)->Eigen::Matrix3d {
                                                           return kdlToEigen(in);
                                                         });
  }

  inline tf2::Transform kdlToTf2(const KDL::Frame& in)
  {
    tf2::Transform out;
    tf2::convert(tf2::kdlToTransform(in).transform, out);
    return out;
  }

  inline tf2::Vector3 kdlToTf2(const KDL::Vector& in)
  {
    tf2::Stamped<tf2::Vector3> out;
    tf2::convert(kdlToMsg(in), out);
    return out;
  }

  inline std::vector<tf2::Vector3> kdlToTf2(const std::vector<KDL::Vector>& in)
  {
    return convertVector<tf2::Vector3, KDL::Vector>(in,
                                                    [](const KDL::Vector& in)->tf2::Vector3 {
                                                      return kdlToTf2(in);
                                                    });
  }

  inline bool addTreeRecursive(KDL::SegmentMap::const_iterator root, const std::string& hook_name, KDL::Tree& tree, const std::string& skip_segments_root="") {
      //const KDL::SegmentMap& skipped_segments=KDL::SegmentMap()) {
    //get iterator for root-segment
    KDL::SegmentMap::const_iterator child;
    //try to add all of root's children
    for (unsigned int i = 0; i < (root->second).children.size(); i++) {
      child = (root->second).children[i];
      if (skip_segments_root!="" && child->first == skip_segments_root) continue;
      //Try to add the child
      if (tree.addSegment((child->second).segment, hook_name)) {
        //if child is added, add all the child's children
        if (!(addTreeRecursive(child, child->first, tree, skip_segments_root)))
          //if it didn't work, return false
          return false;
      } else
        //If the child could not be added, return false
        return false;
    }
    return true;
  }

  /**
   * Extract a tree having segment_name as root. Only child segments of
   * segment_name are added to the new tree.
   *
   * @param segment_name the name of the segment to be used as root
   * of the new tree
   * @param tree_origin the original tree to exctract sub-tree from
   *
   * @return subtree the resulting sub-tree
   */
  inline KDL::Tree getSubTree(const std::string& segment_name, const KDL::Tree& tree_origin)
  {
    KDL::Tree subtree;
    const KDL::SegmentMap& segments = tree_origin.getSegments();
    //check if segment_name exists
    KDL::SegmentMap::const_iterator root = segments.find(segment_name);
    if (root == segments.end())
      return subtree;
    //init the subtree, segment_name is the new root.
    subtree = KDL::Tree(root->first);

    addTreeRecursive(root, segment_name, subtree);
    return subtree;
  }

  /**
   * Remove a sub-tree having segment_name as root from tree_origin.
   *
   * @param segment_name the name of the segment to be used as root
   * of the removed sub-tree
   * @param tree_origin the original tree to remove sub-tree from
   *
   * @return tree_trimed the resulting tree
   */
  inline KDL::Tree removeSegmentsFrom(const std::string& segment_name, const KDL::Tree& tree_origin)
  {
    KDL::Tree tree_trimed;
    std::string origin_root_name = tree_origin.getRootSegment()->first;
    if (segment_name == origin_root_name) return tree_trimed; // can not remove root segment
    if (tree_origin.getSegments().find(segment_name) == tree_origin.getSegments().end()) return tree_trimed; // can not remove non existent segment
    tree_trimed = KDL::Tree(tree_origin.getRootSegment()->first);
    addTreeRecursive(tree_origin.getSegments().find(origin_root_name), origin_root_name, tree_trimed, segment_name);
    return tree_trimed;
  }

} //namespace aerial_robot_model
